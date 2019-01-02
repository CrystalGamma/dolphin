// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

#include <cstddef>
#include <optional>

#include "Core/PowerPC/JitTiered/TieredPPC64.h"

void PPC64BaselineCompiler::RestoreRegistersReturn(u32 saved_regs)
{
  ADDI(R1, R1, 32 + 8 * saved_regs);
  relocations.push_back(B(BR_NORMAL, offsets.restore_gpr_return + (18 - saved_regs) * 4));
}

void PPC64BaselineCompiler::EmitCommonRoutines()
{
  // these three are recommended sequences from the ELFv2 spec
  offsets.save_gprs = instructions.size() * 4;
  for (s16 reg = 14; reg < 32; ++reg)
  {
    STD(static_cast<GPR>(R0 + reg), R1, 8 * (reg - 32));
  }
  BCLR();

  offsets.restore_gpr_return = instructions.size() * 4;
  for (s16 reg = 14; reg < 29; ++reg)
  {
    LD(static_cast<GPR>(R0 + reg), R1, 8 * (reg - 32));
  }
  LD(R0, R1, 16);
  LD(R29, R1, -24);
  MTSPR(SPR_LR, R0);
  LD(R30, R1, -16);
  LD(R31, R1, -8);
  BCLR();

  offsets.restore_gpr = instructions.size() * 4;
  for (s16 reg = 14; reg < 32; ++reg)
  {
    LD(static_cast<GPR>(R0 + reg), R12, 8 * (reg - 32));
  }
  BCLR();

  // this one assumes there is still a stack frame, but all nonvolatile registers have been restored
  // already
  offsets.exception_exit = instructions.size() * 4;
  // call CheckExceptions
  LD(R12, TOC, s16(s32(offsetof(TableOfContents, check_exceptions)) - 0x4000));
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
  // return 0 (no overrun)
  LoadUnsignedImmediate(ARG1, 0);
  LD(R1, R1, 0);
  LD(R0, R1, 16);
  MTSPR(SPR_LR, R0);
  BCLR();

  offsets.end = instructions.size() * 4;
}

void PPC64BaselineCompiler::Compile(u32 addr,
                                    const std::vector<UGeckoInstruction>& guest_instructions)
{
  address = addr;
  u32 saved_regs = 3;
  // allocate stack frame, save caller registers
  MFSPR(R0, SPR_LR);
  STD(R0, R1, 16);
  relocations.push_back(B(BR_LINK, offsets.save_gprs + (18 - saved_regs) * 4));
  STD(R1, R1, -32 - 8 * saved_regs, UPDATE);
  // copy our variables
  MoveReg(PPCSTATE, R5);
  ADDI(TOC, R6, 0x4000);

  INFO_LOG(DYNA_REC, "Compiling code at %08x", address);
  for (u32 index = 0; index < guest_instructions.size(); ++index)
  {
    auto inst = guest_instructions[index];
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    INFO_LOG(DYNA_REC, "%08x: %08x %s", address, inst.hex, opinfo->opname);
    // set PC + NPC
    LoadUnsignedImmediate(SCRATCH1, address);
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    ADDI(SCRATCH1, SCRATCH1, 4);
    STW(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
    // decrement downcount (buffered)
    downcount += opinfo->numCycles;
    if (opinfo->flags & FL_USE_FPU && !float_check.has_value())
    {
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, msr)));
      // test for FP bit
      ANDI_Rc(SCRATCH1, SCRATCH1, 1 << 13);
      float_check = {BC(BR_TRUE, CR0 + EQ), downcount};
    }

    if (inst.OPCD == 14 || inst.OPCD == 15)
    {
      // addi, addis
      if (inst.RA != 0)
      {
        LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
        DFormInstruction(inst.OPCD, SCRATCH1, SCRATCH1, inst.SIMM_16);
      }
      else if (inst.OPCD == 14)
      {
        LoadSignedImmediate(SCRATCH1, inst.SIMM_16);
      }
      else
      {
        LoadSignedImmediate(SCRATCH1, s32(inst.SIMM_16) << 16);
      }
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RD));
    }
    else if (inst.OPCD >= 24 && inst.OPCD <= 29)
    {
      // ori(s), xori(s), andi(s).
      LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RD));
      DFormInstruction(inst.OPCD, SCRATCH1, SCRATCH1, inst.UIMM);
      if (inst.OPCD == 28 || inst.OPCD == 29)
      {
        EXTSW(SCRATCH1, SCRATCH1);
        // FIXME: implement SO emulation?
        STD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val)));
      }
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
    }
    else if (inst.OPCD == 7)
    {
      // mulli
      LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
      MULLI(SCRATCH1, SCRATCH1, inst.SIMM_16);
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RD));
    }
    else if (inst.OPCD == 20 || inst.OPCD == 21)
    {
      // rlwimix, rlwinmx
      if (inst.OPCD == 20)
      {
        LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
        LWZ(SCRATCH2, PPCSTATE, GPROffset(inst.RD));
      }
      else
      {
        LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RD));
      }
      // replace the registers with our own, clear Rc and leave the rest as is
      this->instructions.push_back((inst.hex & 0xfc00fffe) |
                                   (u32(inst.OPCD == 20 ? SCRATCH2 : SCRATCH1) << 21) |
                                   (u32(SCRATCH1) << 16));
      if (inst.Rc)
      {
        EXTSW(SCRATCH1, SCRATCH1);
        // FIXME: implement SO emulation?
        STD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val)));
      }
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
    }
    else if (inst.hex == 0x4182fff8 && index >= 2 &&
             (guest_instructions[index - 2].hex >> 16) == 0x800d &&
             guest_instructions[index - 1].hex == 0x28000000)
    {
      // idle skip as detected in Interpreter
      ERROR_LOG(DYNA_REC, "compiling idle skip @ %08x", address);
      LD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val)));
      CMPLI(CR0, CMP_WORD, SCRATCH1, 0);
      jumps.push_back({BC(BR_TRUE, CR0 + EQ), address - 8, downcount, SKIP, 0});
    }
    else
    {
      FallbackToInterpreter(inst, *opinfo);
    }
    address += 4;
  }
  LoadUnsignedImmediate(SCRATCH1, address);
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
  ADDI(SCRATCH2, SCRATCH2, -s16(downcount));
  STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
  bool block_end = JitTieredGeneric::IsRedispatchInstruction(guest_instructions.back());
  LoadUnsignedImmediate(ARG1, block_end ? 0 : JitTieredGeneric::BLOCK_OVERRUN);
  RestoreRegistersReturn(saved_regs);

  std::vector<FixupBranch> check_exc_exits;
  if (float_check.has_value())
  {
    SetBranchTarget(float_check->branch);
    LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
    ORI(SCRATCH1, SCRATCH1, u16(EXCEPTION_FPU_UNAVAILABLE));
    STW(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(float_check->downcount));
    STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    check_exc_exits.push_back(B());
  }

  for (auto eexit : exc_exits)
  {
    SetBranchTarget(eexit.branch);
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(eexit.downcount));
    STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    check_exc_exits.push_back(B());
  }

  for (auto branch : check_exc_exits)
  {
    SetBranchTarget(branch);
  }
  // call CheckExceptions
  LD(R12, TOC, s16(s32(offsetof(TableOfContents, check_exceptions)) - 0x4000));
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
  auto exc_jump = B();

  std::vector<FixupBranch> store_pc_exits;
  for (auto jexit : jmp_exits)
  {
    SetBranchTarget(jexit.branch);
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(jexit.downcount));
    store_pc_exits.push_back(B());
  }

  for (auto jump : jumps)
  {
    SetBranchTarget(jump.branch);
    if (jump.flags & SKIP)
    {
      // call CoreTiming::Idle
      LD(R12, TOC, s16(s32(offsetof(TableOfContents, idle)) - 0x4000));
      MTSPR(SPR_CTR, R12);
      BCCTR();
    }
    // decrement *after* skip – important for timing equivalency to Generic
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(jump.downcount));
    LoadUnsignedImmediate(SCRATCH1, jump.address);
    store_pc_exits.push_back(B());
  }

  for (auto branch : store_pc_exits)
  {
    SetBranchTarget(branch);
  }
  STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
  SetBranchTarget(exc_jump);
  // store NPC to PC, store downcount
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  LoadUnsignedImmediate(ARG1, 0);
  LD(R29, R1, 32);
  LD(R30, R1, 40);
  LD(R31, R1, 48);
  ADDI(R1, R1, 56);
  LD(R0, R1, 16);
  MTSPR(SPR_LR, R0);
  BCLR();
  return;
}

void PPC64BaselineCompiler::RelocateAll(u32 offset)
{
  for (auto branch : relocations)
  {
    // this assumes there are the common routines at both offset 0 and 1^26, so wrapping the branch
    // offsets around is fine
    Relocate(branch, -Common::BitCast<s32>(offset + u32(branch * 4)));
  }
}

void PPC64BaselineCompiler::FallbackToInterpreter(UGeckoInstruction inst, GekkoOPInfo& opinfo)
{
  u32 fallback_index;
  switch (inst.OPCD)
  {
  case 4:
    fallback_index = 64 + inst.SUBOP10;
    break;
  case 19:
    fallback_index = 64 + 1024 + inst.SUBOP10;
    break;
  case 31:
    fallback_index = 64 + 2 * 1024 + inst.SUBOP10;
    break;
  case 63:
    fallback_index = 64 + 3 * 1024 + inst.SUBOP10;
    break;
  case 59:
    fallback_index = 64 + 4 * 1024 + inst.SUBOP5;
    break;
  default:
    fallback_index = inst.OPCD;
  }
  // load interpreter routine
  LD(R12, TOC, s16(s32(offsetof(TableOfContents, fallback_table) + 8 * fallback_index) - 0x4000));
  MTSPR(SPR_CTR, R12);
  // load instruction value into first argument register
  LoadUnsignedImmediate(ARG1, inst.hex);
  // do the call
  BCCTR();
  // check for exceptions
  LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  ANDI_Rc(SCRATCH1, SCRATCH1, u16(JitTieredGeneric::EXCEPTION_SYNC));
  exc_exits.push_back({BC(BR_FALSE, CR0 + EQ), downcount});
  // load NPC into SCRATCH1 and return if it's ≠ PC + 4
  LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
  XORIS(SCRATCH2, SCRATCH1, u16((address + 4) >> 16));
  CMPLI(CR0, CMP_WORD, SCRATCH2, u16((address + 4) & 0xffff));
  jmp_exits.push_back({BC(BR_FALSE, CR0 + EQ), downcount});
}
