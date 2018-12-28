// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

#include <cstddef>
#include <optional>

#include "Core/PowerPC/JitTiered/TieredPPC64.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/PowerPC.h"

constexpr s16 GPROffset(u32 i)
{
  return s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * i);
}

void PPC64BaselineCompiler::Compile(u32 address,
                                    const std::vector<UGeckoInstruction>& guest_instructions)
{
  constexpr GPR PPCSTATE = R30;
  constexpr GPR TOC = R29;
  constexpr GPR SCRATCH1 = R7;
  constexpr GPR SCRATCH2 = R8;
  constexpr GPR SAVED1 = R31;
  constexpr GPR ARG1 = R3;
  constexpr s16 OFF_PC = s16(offsetof(PowerPC::PowerPCState, pc));
  constexpr s16 OFF_DOWNCOUNT = s16(offsetof(PowerPC::PowerPCState, downcount));
  constexpr s16 OFF_EXCEPTIONS = s16(offsetof(PowerPC::PowerPCState, Exceptions));
  // allocate stack frame
  MFSPR(R0, SPR_LR);
  STD(R0, R1, 16);
  STD(R1, R1, -56, UPDATE);
  // save caller registers and replace them with our values
  STD(R29, R1, 32);
  STD(R30, R1, 40);
  STD(R31, R1, 48);
  MoveReg(PPCSTATE, R5);
  ADDI(TOC, R6, 0x4000);

  struct ExceptionExit
  {
    FixupBranch branch;
    u32 address;
    u16 downcount;
  };

  INFO_LOG(DYNA_REC, "Compiling code at %08x", address);
  std::vector<FixupBranch> exits;
  std::vector<FixupBranch> jmp_exits;
  std::vector<FixupBranch> exc_exits;
  std::optional<FixupBranch> float_check;
  for (auto inst : guest_instructions)
  {
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    INFO_LOG(DYNA_REC, "%08x: %08x %s", address, inst.hex, opinfo->opname);
    // set PC + NPC
    LoadUnsignedImmediate(SCRATCH1, address);
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    ADDI(SCRATCH1, SCRATCH1, 4);
    STW(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
    if (opinfo->flags & FL_USE_FPU && !float_check.has_value())
    {
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, msr)));
      // test for FP bit
      ANDI_Rc(SCRATCH1, SCRATCH1, 1 << 13);
      float_check.emplace(ConditionalBranch(BR_TRUE, CR0 + EQ));
    }
    u32 index;
    switch (inst.OPCD)
    {
    case 4:
      index = 64 + inst.SUBOP10;
      break;
    case 19:
      index = 64 + 1024 + inst.SUBOP10;
      break;
    case 31:
      index = 64 + 2 * 1024 + inst.SUBOP10;
      break;
    case 63:
      index = 64 + 3 * 1024 + inst.SUBOP10;
      break;
    case 59:
      index = 64 + 4 * 1024 + inst.SUBOP5;
      break;
    default:
      index = inst.OPCD;
    }
    // load interpreter routine
    LD(R12, TOC,
       s16(s32(offsetof(JitTieredPPC64::TableOfContents, fallback_table) + 8 * index) - 0x4000));
    MTSPR(SPR_CTR, R12);
    // load instruction value into first argument register
    LoadUnsignedImmediate(ARG1, inst.hex);
    // do the call
    BCCTR();
    // decrement downcount
    LWZ(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH1, SCRATCH1, -s16(opinfo->numCycles));
    STW(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
    // check for exceptions
    LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
    ANDI_Rc(SCRATCH1, SCRATCH1, u16(JitTieredGeneric::EXCEPTION_SYNC));
    exc_exits.push_back(ConditionalBranch(BR_FALSE, CR0 + EQ));
    // load NPC into SCRATCH1 and return if it's ≠ PC + 4
    LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
    XORIS(SCRATCH2, SCRATCH1, u16((address + 4) >> 16));
    CMPLI(CR0, CMP_WORD, SCRATCH2, u16((address + 4) & 0xffff));
    jmp_exits.emplace_back(ConditionalBranch(BR_FALSE, CR0 + EQ));
    address += 4;
  }
  LoadUnsignedImmediate(SCRATCH1, address);
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  LoadUnsignedImmediate(ARG1, JitTieredGeneric::BLOCK_OVERRUN);
  auto exit_branch = Jump();

  if (float_check.has_value())
  {
    SetBranchTarget(*float_check);
    LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
    ORI(SCRATCH1, SCRATCH1, u16(EXCEPTION_FPU_UNAVAILABLE));
    STW(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  }
  // fall through to exception check

  for (auto branch : exc_exits)
  {
    SetBranchTarget(branch);
  }
  // call CheckExceptions
  LD(R12, TOC, s16(s32(offsetof(JitTieredPPC64::TableOfContents, check_exceptions)) - 0x4000));
  MTSPR(SPR_CTR, R12);
  BCCTR();
  // clear synchronous exceptions (TODO: check if this is really necessary; Interpreter doesn't do
  // it, but I've had some bugs when I didn't do it in my code)
  LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  LoadSignedImmediate(SCRATCH2, s16(Common::BitCast<s32>(~JitTieredGeneric::EXCEPTION_SYNC)));
  AND(SCRATCH1, SCRATCH1, SCRATCH2);
  STW(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  // update PC
  LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
  // fall through: exceptions are treated as jumps (since they usually are)

  for (auto branch : jmp_exits)
  {
    SetBranchTarget(branch);
  }
  // store NPC to PC
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  LoadUnsignedImmediate(ARG1, 0);
  for (auto branch : exits)
  {
    SetBranchTarget(branch);
  }
  SetBranchTarget(exit_branch);
  LD(R29, R1, 32);
  LD(R30, R1, 40);
  LD(R31, R1, 48);
  ADDI(R1, R1, 56);
  LD(R0, R1, 16);
  MTSPR(SPR_LR, R0);
  BCLR();
  return;
}
