// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

#include <cstddef>
#include <optional>

#include "Common/Assert.h"
#include "Core/PowerPC/JitTiered/TieredPPC64.h"

void PPC64BaselineCompiler::RestoreRegistersReturn(u32 saved_regs)
{
  ADDI(R1, R1, 32 + 8 * saved_regs);
  relocations.push_back(B(BR_NORMAL, offsets.restore_gpr_return + (18 - saved_regs) * 4));
}

void PPC64BaselineCompiler::RestoreRegisters(u32 saved_regs)
{
  ADDI(R12, R1, 32 + 8 * saved_regs);
  relocations.push_back(B(BR_LINK, offsets.restore_gpr + (18 - saved_regs) * 4));
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

  bool float_checked = false;
  bool omit_epilogue = false;
  INFO_LOG(DYNA_REC, "Compiling code at %08x", address);
  for (u32 index = 0; index < guest_instructions.size(); ++index)
  {
    auto inst = guest_instructions[index];
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    INFO_LOG(DYNA_REC, "%08x: %08x %s", address, inst.hex, opinfo->opname);
    // decrement downcount (buffered)
    downcount += opinfo->numCycles;
    if (opinfo->flags & FL_USE_FPU && !float_checked)
    {
      LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, msr)));
      // test for FP bit
      ANDI_Rc(SCRATCH1, SCRATCH1, 1 << 13);
      exits.emplace_back(BC(BR_TRUE, CR0 + EQ),
                         Exit{address, downcount, EXCEPTION | RAISE_FPU_EXC, 0});
      float_checked = true;
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
             (guest_instructions[index - 1].hex & 0xfbffffff) == 0x28000000)
    {
      // idle skip as detected in Interpreter
      ERROR_LOG(DYNA_REC, "compiling idle skip (wait-on-word) @ %08x", address);
      LD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val)));
      CMPLI(CR0, CMP_WORD, SCRATCH1, 0);
      exits.emplace_back(BC(BR_TRUE, CR0 + EQ), Exit{address - 8, downcount, SKIP, 0});
    }
    else if (inst.hex == 0x48000000)
    {
      // idle skip as detected in Interpreter
      ERROR_LOG(DYNA_REC, "compiling idle skip (empty loop) @ %08x", address);
      WriteExit({address, downcount, SKIP, 0});
      omit_epilogue = true;
      break;
    }
    else if (inst.OPCD == 18)
    {
      // unconditional branch
      u32 base = inst.AA ? 0 : address;
      u32 target = base + u32(Common::BitCast<s32>(inst.LI << 8) >> 6);
      WriteExit({target, downcount, inst.LK ? LINK : JUMP, address + 4});
      omit_epilogue = true;
      break;
    }
    else if (inst.OPCD == 16 || (inst.OPCD == 19 && inst.SUBOP5 == 16))
    {
      BCX(inst, *opinfo);
    }
    else if ((inst.OPCD >= 32 && inst.OPCD <= 45) || (inst.OPCD == 31 && inst.SUBOP5 == 23))
    {
      LoadStore(inst, *opinfo);
    }
    else if (inst.OPCD == 31 && (inst.SUBOP10 == 28 || inst.SUBOP10 == 444))
    {
      // reg-reg bitops (andx, orx for now)
      LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RB));
      LWZ(SCRATCH2, PPCSTATE, GPROffset(inst.RD));
      // these are well-defined for all inputs, so let's just override the registers with ours
      XFormInstruction(31, SCRATCH1, SCRATCH1, SCRATCH2, inst.SUBOP10);
      if (inst.Rc)
      {
        EXTSW(SCRATCH1, SCRATCH1);
        // FIXME: implement SO emulation?
        STD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val)));
      }
      STW(SCRATCH1, PPCSTATE, GPROffset(inst.RA));
    }
    else
    {
      FallbackToInterpreter(inst, *opinfo);
    }
    address += 4;
  }
  if (!omit_epilogue)
  {
    LoadUnsignedImmediate(SCRATCH1, address);
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    STW(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(downcount));
    STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    const bool block_end = JitTieredGeneric::IsRedispatchInstruction(guest_instructions.back());
    LoadUnsignedImmediate(ARG1, block_end ? 0 : JitTieredGeneric::BLOCK_OVERRUN);
    RestoreRegistersReturn(saved_regs);
  }

  for (auto fexit : fallback_exits)
  {
    SetBranchTarget(fexit.store_pc);
    STW(SCRATCH1, PPCSTATE, OFF_PC);
    SetBranchTarget(fexit.leave_pc);
    LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    ADDI(SCRATCH2, SCRATCH2, -s16(fexit.downcount));
    STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
    LoadUnsignedImmediate(ARG1, 0);
    RestoreRegistersReturn(saved_regs);
  }

  for (auto& pair : exits)
  {
    SetBranchTarget(pair.first);
    WriteExit(pair.second);
  }
}

void PPC64BaselineCompiler::WriteExit(const Exit& jump)
{
  const u32 saved_regs = 3;
  if (jump.flags & RAISE_FPU_EXC)
  {
    LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
    ORI(SCRATCH1, SCRATCH1, u16(EXCEPTION_FPU_UNAVAILABLE));
    STW(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  }
  if (jump.flags & SKIP)
  {
    // call CoreTiming::Idle
    LD(R12, TOC, s16(s32(offsetof(TableOfContents, idle)) - 0x4000));
    MTSPR(SPR_CTR, R12);
    BCCTR();
  }
  if (jump.flags & JUMPSPR)
  {
    LWZ(SCRATCH1, PPCSTATE, SPROffset(jump.address));
  }
  else
  {
    LoadUnsignedImmediate(SCRATCH1, jump.address);
  }
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  if (jump.flags & EXCEPTION)
  {
    ADDI(SCRATCH1, SCRATCH1, 4);
  }
  STW(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
  if (jump.flags & LINK)
  {
    LoadUnsignedImmediate(SCRATCH2, jump.link_address);
    STW(SCRATCH2, PPCSTATE, SPROffset(SPR_LR));
  }
  // decrement *after* skip – important for timing equivalency to Generic
  LWZ(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
  ADDI(SCRATCH2, SCRATCH2, -s16(jump.downcount));
  STW(SCRATCH2, PPCSTATE, OFF_DOWNCOUNT);
  LoadUnsignedImmediate(ARG1, 0);
  RestoreRegistersReturn(saved_regs);
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
  // set PC + NPC
  LoadUnsignedImmediate(SCRATCH1, address);
  STW(SCRATCH1, PPCSTATE, OFF_PC);
  ADDI(SCRATCH1, SCRATCH1, 4);
  STW(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
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
  auto exc_exit = BC(BR_FALSE, CR0 + EQ);
  // load NPC into SCRATCH1 and return if it's ≠ PC + 4
  LWZ(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, npc)));
  XORIS(SCRATCH2, SCRATCH1, u16((address + 4) >> 16));
  CMPLI(CR0, CMP_WORD, SCRATCH2, u16((address + 4) & 0xffff));
  auto jump_exit = BC(BR_FALSE, CR0 + EQ);
  fallback_exits.push_back({jump_exit, exc_exit, downcount});
}

void PPC64BaselineCompiler::BCX(UGeckoInstruction inst, GekkoOPInfo& opinfo)
{
  if ((inst.BO & (BO_DONT_CHECK_CONDITION | BO_DONT_DECREMENT_FLAG)) == 0)
  {
    WARN_LOG(DYNA_REC, "CR+CTR branch %08x @ %08x", inst.hex, address);
  }
  bool inverted = false;
  u32 bit = 0;
  if (!(inst.BO & BO_DONT_CHECK_CONDITION))
  {
    const bool branch_on_true = inst.BO & BO_BRANCH_IF_TRUE;
    LD(SCRATCH1, PPCSTATE, s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * (inst.BI / 4)));
    switch (inst.BI % 4)
    {
    case LT:
      INFO_LOG(DYNA_REC, "LT branch @ %08x", address);
      RLDICL(SCRATCH1, SCRATCH1, 2, 63, RC);
      inverted = branch_on_true;
      bit = CR0 + EQ;
      break;
    case GT:
      INFO_LOG(DYNA_REC, "GT branch @ %08x", address);
      CMPI(CR0, CMP_DWORD, SCRATCH1, 0);
      inverted = !branch_on_true;
      bit = CR0 + GT;
      break;
    case EQ:
      INFO_LOG(DYNA_REC, "EQ branch @ %08x", address);
      CMPLI(CR0, CMP_WORD, SCRATCH1, 0);
      inverted = !branch_on_true;
      bit = CR0 + EQ;
      break;
    case SO:
      INFO_LOG(DYNA_REC, "SO branch @ %08x", address);
      TW();
      RLDICL(SCRATCH1, SCRATCH1, 3, 63, RC);
      inverted = !branch_on_true;
      bit = CR0 + EQ;
    }
  }
  if (!(inst.BO & BO_DONT_DECREMENT_FLAG))
  {
    LWZ(SCRATCH1, PPCSTATE, SPROffset(SPR_CTR));
    ADDI(SCRATCH1, SCRATCH1, -1);
    STW(SCRATCH1, PPCSTATE, SPROffset(SPR_CTR));
    CMPLI(CR1, CMP_WORD, SCRATCH1, 0);
    bool branch_on_zero = inst.BO & BO_BRANCH_IF_CTR_0;
    if (inst.BO & BO_DONT_CHECK_CONDITION)
    {
      bit = CR1 + EQ;
      inverted = !branch_on_zero;
    }
    else if (!inverted && branch_on_zero)
    {
      CRAND(bit, CR1 + EQ, bit);
    }
    else if (branch_on_zero)
    {
      CRANDC(bit, CR1 + EQ, bit);
      inverted = false;
    }
    else if (!inverted)
    {
      CRANDC(bit, bit, CR1 + EQ);
    }
    else
    {
      CRNOR(bit, bit, CR1 + EQ);
      inverted = false;
    }
  }
  FixupBranch branch;
  if ((~inst.BO & (BO_DONT_CHECK_CONDITION | BO_DONT_DECREMENT_FLAG)) == 0)
  {
    if (inst.OPCD == 16)
    {
      WARN_LOG(DYNA_REC, "unconditional BC @ %08x", address);
    }
    branch = B();
  }
  else
  {
    branch = BC(inverted ? BR_FALSE : BR_TRUE, bit);
  }
  if (inst.OPCD == 16)
  {
    u32 jump_base = inst.AA ? 0 : address;
    // man, this sign-extension looks ugly
    u32 target = jump_base + Common::BitCast<u32>(s32(Common::BitCast<s16>(u16(inst.BD << 2))));
    INFO_LOG(DYNA_REC, "target: %08x", target);
    exits.emplace_back(branch, Exit{target, downcount, inst.LK ? LINK : JUMP, address + 4});
  }
  else
  {
    u32 reg = inst.SUBOP10 == 16 ? SPR_LR : SPR_CTR;
    exits.emplace_back(branch,
                       Exit{reg, downcount, JUMPSPR | (inst.LK ? LINK : JUMP), address + 4});
  }
}

void PPC64BaselineCompiler::LoadStore(UGeckoInstruction inst, GekkoOPInfo& opinfo)
{
  u32 op;
  const bool is_indexed = inst.OPCD == 31;
  if (is_indexed)
  {
    op = inst.SUBOP10 >> 5;
  }
  else
  {
    op = inst.OPCD - 32;
  }
  if (op > 13)
  {
    FallbackToInterpreter(inst, opinfo);
    return;
  }
  const bool is_store = op & 4;
  const bool is_update = op & 1;
  s16 offset = 0;
  switch (op >> 1)
  {
  case 0:
    offset = s16(offsetof(TableOfContents, load_word) - 0x4000);
    break;
  case 1:
    offset = s16(offsetof(TableOfContents, load_byte) - 0x4000);
    break;
  case 2:
    offset = s16(offsetof(TableOfContents, store_word) - 0x4000);
    break;
  case 3:
    offset = s16(offsetof(TableOfContents, store_byte) - 0x4000);
    break;
  case 4:
    offset = s16(offsetof(TableOfContents, load_hword) - 0x4000);
    break;
  case 5:
    offset = s16(offsetof(TableOfContents, load_hword_sext) - 0x4000);
    break;
  case 6:
    offset = s16(offsetof(TableOfContents, store_hword) - 0x4000);
    break;
  default:
    ERROR_LOG(DYNA_REC, "unexpected opcode %08x @ %08x", inst.hex, address);
  }
  ASSERT(offset != 0);
  if (inst.RA != 0)
  {
    // calculate address
    LWZ(SAVED1, PPCSTATE, GPROffset(inst.RA));
    if (is_indexed)
    {
      LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RB));
      ADD(SAVED1, SAVED1, SCRATCH1);
    }
    else
    {
      ADDI(SAVED1, SAVED1, inst.SIMM_16);
    }
    RLDICL(SAVED1, SAVED1, 0, 32);
  }
  else
  {
    ASSERT(!is_update);
    if (is_indexed)
    {
      LWZ(SCRATCH1, PPCSTATE, GPROffset(inst.RB));
    }
    else
    {
      LoadUnsignedImmediate(SAVED1, Common::BitCast<u32>(s32(inst.SIMM_16)));
    }
  }
  // call the MMU function
  LD(R12, TOC, offset);
  if (!is_store)
  {
    MoveReg(ARG1, SAVED1);
  }
  else
  {
    LWZ(ARG1, PPCSTATE, GPROffset(inst.RD));
    // make sure it's properly zero-extended
    if ((op >> 1) == 3)
    {
      RLDICL(ARG1, ARG1, 0, 56);
    }
    else if ((op >> 1) == 6)
    {
      RLDICL(ARG1, ARG1, 0, 48);
    }
    MoveReg(ARG2, SAVED1);
  }
  MTSPR(SPR_CTR, R12);
  BCCTR();
  // check for exceptions
  LWZ(SCRATCH1, PPCSTATE, OFF_EXCEPTIONS);
  ANDI_Rc(SCRATCH1, SCRATCH1, u16(JitTieredGeneric::EXCEPTION_SYNC));
  exits.emplace_back(BC(BR_FALSE, CR0 + EQ), Exit{address, downcount, EXCEPTION, 0});
  // maintain GPR file
  if (is_update)
  {
    ASSERT(is_store || inst.RA != inst.RD);
    STW(SAVED1, PPCSTATE, GPROffset(inst.RA));
  }
  if (!is_store)
  {
    STW(ARG1, PPCSTATE, GPROffset(inst.RD));
  }
}
