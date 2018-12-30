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
  // allocate stack frame
  MFSPR(R0, SPR_LR);
  STD(R0, R1, 16);
  STD(R1, R1, -56, UPDATE);
  // save caller registers and replace them with our values
  STD(R29, R1, 32);
  STD(R30, R1, 40);
  STD(R31, R1, 48);
  MoveReg(PPCSTATE, R5);
  MoveReg(TOC, R6);

  struct ExceptionExit
  {
    FixupBranch branch;
    u32 address;
    u16 downcount;
  };

  INFO_LOG(DYNA_REC, "Compiling code at %08x", address);
  std::vector<FixupBranch> exits;
  LD(SAVED1, TOC, s16(offsetof(JitTieredPPC64::TableOfContents, do_instruction)));
  for (auto inst : guest_instructions)
  {
    MoveReg(R12, SAVED1);
    MTSPR(SPR_CTR, SAVED1);
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    INFO_LOG(DYNA_REC, "%08x: %08x %s", address, inst.hex, opinfo->opname);
    // load instruction value into first argument register
    LoadUnsignedImmediate(ARG1, inst.hex);
    // do the call
    BCCTR();
    CMPLI(CR0, CMP_WORD, ARG1, JitTieredGeneric::BLOCK_OVERRUN);
    exits.push_back(ConditionalBranch(BR_FALSE, CR0 + EQ));
  }
  for (auto branch : exits)
  {
    SetBranchTarget(branch);
  }
  LD(R29, R1, 32);
  LD(R30, R1, 40);
  LD(R31, R1, 48);
  ADDI(R1, R1, 56);
  LD(R0, R1, 16);
  MTSPR(SPR_LR, R0);
  BCLR();
  return;
}
