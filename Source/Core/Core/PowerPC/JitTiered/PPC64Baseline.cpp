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
  constexpr GPR TOC = R31;
  constexpr GPR SCRATCH1 = R7;
  constexpr GPR SCRATCH2 = R7;
  constexpr GPR ARG1 = R3;
  constexpr s16 OFF_PC = s16(offsetof(PowerPC::PowerPCState, pc));
  constexpr s16 OFF_DOWNCOUNT = s16(offsetof(PowerPC::PowerPCState, downcount));
  // allocate stack frame
  MFSPR(R0, SPR_LR);
  STD(R0, R1, 16);
  STD(R1, R1, -48, UPDATE);
  // save caller registers and replace them with our values
  STD(R30, R1, 32);
  STD(R31, R1, 40);
  TW();
  MoveReg(PPCSTATE, R5);
  MoveReg(TOC, R6);

  struct ExceptionExit
  {
    FixupBranch branch;
    u32 address;
    u16 downcount;
  };

  std::optional<FixupBranch> float_check;
  std::vector<ExceptionExit> exception_exits;
  std::vector<std::pair<FixupBranch, u16>> jump_exits;
  std::vector<FixupBranch> straight_exits;
  u16 downcount = 0;
  for (auto inst : guest_instructions)
  {
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    u32 flags = opinfo->flags;
    if ((flags & FL_USE_FPU) && !float_check)
    {
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, msr)));
      // test for FP bit
      ANDI_Rc(SCRATCH1, SCRATCH1, 1 << 13);
      float_check.emplace(ConditionalBranch(BR_FALSE, CR0 + GT));
    }
    if (inst.OPCD == 14)
    {
      // addi
      if (inst.RA != 0)
      {
        LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
        ADDI(SCRATCH1, SCRATCH1, inst.SIMM_16);
      }
      else
      {
        LoadSignedImmediate(SCRATCH1, inst.SIMM_16);
      }
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RD));
      downcount += opinfo->numCycles;
    }
    else
    {
      // interpreter fallback
      u32 index;
      switch (inst.OPCD)
      {
      case 4:
        index = 64 + inst.SUBOP10;
        break;
      case 19:
        index = 1088 + inst.SUBOP10;
        break;
      case 31:
        index = 2112 + inst.SUBOP10;
        break;
      case 63:
        index = 3136 + inst.SUBOP10;
        break;
      default:
        index = inst.OPCD;
      }
      // load function address into CTR
      LD(SCRATCH1, TOC, s16(offsetof(JitTieredPPC64::TableOfContents, fallback_table) + 8 * index));
      MTSPR(SPR_CTR, SCRATCH1);
      // load instruction value into first argument register
      LoadUnsignedImmediate(ARG1, Common::BitCast<s32>(inst.hex));
      // do the call
      BCCTR();

      downcount += opinfo->numCycles;
      // check for exceptions
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, Exceptions)));
      ANDI_Rc(SCRATCH1, SCRATCH1, s16(Common::BitCast<s32>(JitTieredGeneric::EXCEPTION_SYNC)));
      exception_exits.push_back({ConditionalBranch(BR_TRUE, CR0 + EQ), address, downcount});
      // compare PC + 4 and NPC (note that SCRATCH1 contains NPC)
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
      // two-instruction compare for equality with 32-bit immediate
      XORIS(SCRATCH2, SCRATCH1, u16((address + 4) >> 16));
      CMPLI(CR0, CMP_WORD, SCRATCH2, u16(address & 0xffff));
      jump_exits.emplace_back(ConditionalBranch(BR_FALSE, CR0 + EQ), downcount);
    }
    address += 4;
  }
  // store PC (TODO: omit after fallback)
  LoadUnsignedImmediate(SCRATCH2, address);
  STW(SCRATCH2, PPCSTATE, OFF_PC);
  // decrement downcount
  LWZ(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
  ADDI(SCRATCH1, SCRATCH1, -s16(downcount));
  STW(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
  // restore caller registers
  LD(R30, R1, 32);
  LD(R31, R1, 40);
  // return BLOCK_OVERRUN
  LoadUnsignedImmediate(ARG1, JitTieredGeneric::BLOCK_OVERRUN);
  BCLR();

  for (auto& eexit : exception_exits)
  {
    SetBranchTarget(eexit.branch);
    // write PC
    LoadUnsignedImmediate(SCRATCH1, eexit.address);
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    // call CheckExceptions
    LD(SCRATCH1, TOC, s16(offsetof(JitTieredPPC64::TableOfContents, check_exceptions)));
    MTSPR(SPR_CTR, SCRATCH1);
    BCCTR();
    // calculate the decremented downcount
    LWZ(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH1, SCRATCH1, -s16(eexit.downcount));
    straight_exits.push_back(Jump());
  }
  for (auto pair : jump_exits)
  {
    SetBranchTarget(pair.first);
    // jump checks load NPC into SCRATCH1 ⇒ store to PC
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    // calculate the decremented downcount
    LWZ(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH1, SCRATCH1, -s16(pair.second));
    straight_exits.push_back(Jump());
  }

  for (auto branch : straight_exits)
  {
    SetBranchTarget(branch);
  }
  // straight exits start with (dirty) downcount in SCRATCH1
  STW(SCRATCH1, PPCSTATE, OFF_DOWNCOUNT);
  // restore caller registers
  LD(R30, R1, 40);
  LD(R31, R1, 32);
  // return 0
  LoadSignedImmediate(ARG1, 0);
  BCLR();
}
