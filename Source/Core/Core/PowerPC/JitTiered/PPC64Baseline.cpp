// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

#include <cstddef>
#include <optional>

#include "Common/Assert.h"
#include "Core/PowerPC/JitTiered/TieredPPC64.h"

// for short register state labels
using namespace PPC64RegCache;

static constexpr s16 SPROffset(u32 i)
{
  return s16(offsetof(PowerPC::PowerPCState, spr) + 4 * i);
}

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
                                    const std::vector<UGeckoInstruction>& guest_instructions,
                                    const std::vector<JitTieredGeneric::Bail>& bails)
{
  address = addr;
  reg_cache = RegisterCache();
  u8 save_regs = 3 + guest_instructions.size() / 2;
  if (guest_instructions.size() > 30)
  {
    save_regs = 18;
  }
  reg_cache.EstablishStackFrame(this, save_regs);

  std::vector<u32> this_inst_bails;
  size_t bails_pos = 0;

  bool float_checked = false;
  bool omit_epilogue = false;
  INFO_LOG(DYNA_REC, "Compiling code at %08x", address);
  for (u32 index = 0; index < guest_instructions.size(); ++index)
  {
    ASSERT(bails_pos >= bails.size() || bails.at(bails_pos).guest_address >= address);
    this_inst_bails.clear();
    while (bails_pos < bails.size() && bails.at(bails_pos).guest_address == address)
    {
      this_inst_bails.push_back(bails.at(bails_pos).status);
      ++bails_pos;
    }
    auto inst = guest_instructions[index];
    GekkoOPInfo* opinfo = PPCTables::GetOpInfo(inst);
    INFO_LOG(DYNA_REC, "%08x: %08x %s", address, inst.hex, opinfo->opname);
    // decrement downcount (buffered)
    downcount += opinfo->numCycles;
    if (opinfo->flags & FL_USE_FPU && !float_checked)
    {
      const GPR scratch = reg_cache.GetScratch(this);
      const GPR ppcs = reg_cache.GetPPCState();
      LWZ(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, msr)));
      // test for FP bit
      ANDI_Rc(scratch, scratch, 1 << 13);
      reg_cache.FreeGPR(scratch);
      exits.emplace_back(BC(BR_TRUE, CR0 + EQ),
                         Exit{reg_cache, address, downcount, EXCEPTION | RAISE_FPU_EXCEPTION, 0});
      float_checked = true;
    }

    if (inst.OPCD == 14 || inst.OPCD == 15)
    {
      const GPR scratch = reg_cache.GetScratch(this);
      // addi, addis
      if (inst.RA != 0)
      {
        const GPR reg = reg_cache.GetGPR(this, DIRTY_R + inst.RA);
        DFormInstruction(inst.OPCD, scratch, reg, inst.SIMM_16);
      }
      else if (inst.OPCD == 14)
      {
        LoadSignedImmediate(scratch, inst.SIMM_16);
      }
      else
      {
        LoadSignedImmediate(scratch, s32(inst.SIMM_16) << 16);
      }
      reg_cache.BindGPR(scratch, DIRTY_R + inst.RD);
    }
    else if (inst.OPCD >= 24 && inst.OPCD <= 29)
    {
      // ori(s), xori(s), andi(s).
      const GPR rs = reg_cache.GetGPR(this, DIRTY_R + inst.RD);
      const GPR ra = reg_cache.GetScratch(this);
      DFormInstruction(inst.OPCD, rs, ra, inst.UIMM);
      // could use ZEXT_R for andi(s)., but setting CR0 involves sign extension anyway
      reg_cache.BindGPR(ra, DIRTY_R + inst.RA);
      if (inst.OPCD == 28 || inst.OPCD == 29)
      {
        reg_cache.SetCR0(this, ra);
      }
    }
    else if (inst.OPCD == 7)
    {
      // mulli
      const GPR reg = reg_cache.GetGPR(this, DIRTY_R + inst.RA);
      const GPR scratch = reg_cache.GetScratch(this);
      MULLI(scratch, reg, inst.SIMM_16);
      reg_cache.BindGPR(scratch, DIRTY_R + inst.RD);
    }
    else if (inst.OPCD == 20 || inst.OPCD == 21)
    {
      // rlwimix, rlwinmx
      GPR rs, ra;
      if (inst.OPCD == 20)
      {
        // rlwimix: RA is in- and output register, and the operation is infallible
        rs = reg_cache.GetGPR(this, DIRTY_R + inst.RD);
        ra = reg_cache.GetGPR(this, (DIRTY_R + inst.RA) | FLAG_GUEST_UNSAVED);
      }
      else
      {
        // rlwinmx
        rs = reg_cache.GetGPR(this, DIRTY_R + inst.RD);
        ra = reg_cache.GetScratch(this);
      }
      // replace the registers with our own, clear Rc and leave the rest as is
      this->instructions.push_back((inst.hex & 0xfc00fffe) | (u32(rs) << 21) | (u32(ra) << 16));
      reg_cache.BindGPR(ra, DIRTY_R + inst.RA);
      if (inst.Rc)
      {
        reg_cache.SetCR0(this, ra);
      }
    }
    else if (inst.hex == 0x4182fff8 && index >= 2 &&
             (guest_instructions[index - 2].hex >> 16) == 0x800d &&
             (guest_instructions[index - 1].hex & 0xfbffffff) == 0x28000000)
    {
      // idle skip as detected in Interpreter
      ERROR_LOG(DYNA_REC, "compiling idle skip (wait-on-word) @ %08x", address);
      GPR scratch = reg_cache.GetScratch(this);
      LD(scratch, reg_cache.GetPPCState(), s16(offsetof(PowerPC::PowerPCState, cr_val)));
      CMPLI(CR0, CMP_WORD, scratch, 0);
      exits.emplace_back(BC(BR_TRUE, CR0 + EQ), Exit{reg_cache, address - 8, downcount, SKIP, 0});
    }
    else if (inst.hex == 0x48000000)
    {
      // idle skip as detected in Interpreter
      ERROR_LOG(DYNA_REC, "compiling idle skip (empty loop) @ %08x", address);
      WriteExit({reg_cache, address, downcount, SKIP, 0});
      omit_epilogue = true;
      break;
    }
    else if (inst.OPCD == 18)
    {
      // unconditional branch
      u32 base = inst.AA ? 0 : address;
      u32 target = base + u32(Common::BitCast<s32>(inst.LI << 8) >> 6);
      WriteExit({reg_cache, target, downcount, inst.LK ? LINK : JUMP, address + 4});
      omit_epilogue = true;
      break;
    }
    else if (inst.OPCD == 16 || (inst.OPCD == 19 && inst.SUBOP5 == 16))
    {
      BCX(inst, *opinfo);
    }
    else if ((inst.OPCD >= 32 && inst.OPCD <= 45) || (inst.OPCD == 31 && inst.SUBOP5 == 23))
    {
      LoadStore(inst, *opinfo, this_inst_bails);
    }
    else if (inst.OPCD == 31 && (inst.SUBOP10 == 28 || inst.SUBOP10 == 444))
    {
      // reg-reg bitops (andx, orx for now)
      GPR rs = reg_cache.GetGPR(this, DIRTY_R + inst.RD);
      GPR rb = reg_cache.GetGPR(this, DIRTY_R + inst.RB);
      GPR ra = reg_cache.GetScratch(this);
      // these are well-defined for all inputs, so let's just override the registers with ours
      XFormInstruction(31, rs, ra, rb, inst.SUBOP10);
      reg_cache.BindGPR(ra, DIRTY_R + inst.RA);
      if (inst.Rc)
      {
        reg_cache.SetCR0(this, ra);
      }
    }
    else if (inst.OPCD == 31 && (inst.SUBOP10 == 266 || inst.SUBOP10 == 40 || inst.SUBOP10 == 235))
    {
      // reg-reg arithmetic ops (addx, subfx, mullwx for now)
      GPR ra = reg_cache.GetGPR(this, DIRTY_R + inst.RA);
      GPR rb = reg_cache.GetGPR(this, DIRTY_R + inst.RB);
      GPR rt = reg_cache.GetScratch(this);
      // these are well-defined for all inputs, so let's just override the registers with ours
      XFormInstruction(31, rt, ra, rb, inst.SUBOP10);
      reg_cache.BindGPR(rt, DIRTY_R + inst.RD);
      if (inst.Rc)
      {
        reg_cache.SetCR0(this, rt);
      }
    }
    else if (inst.OPCD == 11)
    {
      // cmpi: sign extension of input is important here
      GPR ra = reg_cache.GetGPR(this, SEXT_R + inst.RA);
      if (inst.SIMM_16 == 0)
      {
        STD(ra, reg_cache.GetPPCState(),
            s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * inst.CRFD));
      }
      else
      {
        GPR scratch = reg_cache.GetScratch(this);
        AddSImm32(scratch, ra, -inst.SIMM_16);
        // do NOT sign-extend the output of compares!
        STD(scratch, reg_cache.GetPPCState(),
            s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * inst.CRFD));
      }
    }
    else if (inst.OPCD == 10)
    {
      // cmpli: zero extension of input is important here
      GPR ra = reg_cache.GetGPR(this, ZEXT_R + inst.RA);
      if (inst.UIMM == 0)
      {
        STD(ra, reg_cache.GetPPCState(),
            s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * inst.CRFD));
      }
      else
      {
        GPR scratch = reg_cache.GetScratch(this);
        AddSImm32(scratch, ra, -s32(inst.UIMM));
        // do NOT sign-extend the output of compares!
        STD(scratch, reg_cache.GetPPCState(),
            s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * inst.CRFD));
      }
    }
    else if ((inst.OPCD == 46 || inst.OPCD == 47) && this_inst_bails.empty())
    {
      // lmw/stmw – FIXME: like the Interpreter version, this implementation does not roll back on
      // failure
      GPR mem_address = reg_cache.GetScratch(this);
      ADDI(mem_address, reg_cache.GetGPR(this, DIRTY_R + inst.RA), inst.SIMM_16);
      // zero-extend the guest address
      RLWINM(mem_address, mem_address, 0, 0, 31);

      // keep all registers below RT
      u32 register_mask = ~((u32(1) << inst.RD) - 1);
      // stmw doesn't modify registers, so no need to invalidate
      reg_cache.ReduceGuestRegisters(this, register_mask, inst.OPCD == 46 ? register_mask : 0);

      GPR scratch = reg_cache.GetScratch(this);
      // write number of registers to load into CTR
      LoadUnsignedImmediate(scratch, 32 - inst.RD);
      MTSPR(SPR_CTR, scratch);
      // scratch now used for address of GPR in ppcState
      GPR ppcs = reg_cache.GetPPCState();
      // reduce the offset by 4 to account for lwzu/stwu updating the address before loading/storing
      ADDI(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * inst.RD - 4));
      GPR base = reg_cache.GetMemoryBase(this);
      GPR value = reg_cache.GetScratch(this);
      // loop:
      if (inst.OPCD == 46)
      {
        exits.emplace_back(this->instructions.size(),
                           Exit{reg_cache, downcount - u32(opinfo->numCycles), FASTMEM_BAIL, 0});
        LWBRX(value, mem_address, base);
        STWU(value, scratch, 4);
      }
      else
      {
        LWZU(value, scratch, 4);
        exits.emplace_back(this->instructions.size(),
                           Exit{reg_cache, downcount - u32(opinfo->numCycles), FASTMEM_BAIL, 0});
        STWBRX(value, mem_address, base);
      }
      ADDI(mem_address, mem_address, 4);
      BC(BR_DEC_NZ, 0, BR_NORMAL, -12);
      // don't need to load the last register again
      reg_cache.BindGPR(value, ZEXT_R + 31);
    }
    else
    {
      FallbackToInterpreter(inst, *opinfo);
    }
    reg_cache.ReleaseRegisters();
    address += 4;
  }
  if (!omit_epilogue)
  {
    GPR ppcs = reg_cache.GetPPCState();
    GPR scratch = reg_cache.GetScratch(this);
    LoadUnsignedImmediate(scratch, address);
    STW(scratch, ppcs, OFF_PC);
    STW(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, npc)));
    LWZ(scratch, ppcs, OFF_DOWNCOUNT);
    ADDI(scratch, scratch, -s16(downcount));
    STW(scratch, ppcs, OFF_DOWNCOUNT);
    const bool block_end = JitTieredGeneric::IsRedispatchInstruction(guest_instructions.back());
    reg_cache.PrepareReturn(this, 1);
    LoadUnsignedImmediate(reg_cache.GetReturnRegister(0),
                          block_end ? 0 : JitTieredGeneric::BLOCK_OVERRUN);
    RestoreRegistersReturn(reg_cache.saved_regs);
  }

  for (auto fexit : fallback_exits)
  {
    SetBranchTarget(fexit.store_pc);
    GPR ppcs = fexit.reg_cache.GetPPCState();
    STW(fexit.pc, ppcs, OFF_PC);
    SetBranchTarget(fexit.leave_pc);
    fexit.reg_cache.ReleaseRegisters();

    GPR scratch = fexit.reg_cache.GetScratch(this);
    LWZ(scratch, ppcs, OFF_DOWNCOUNT);
    ADDI(scratch, scratch, -s16(fexit.downcount));
    STW(scratch, ppcs, OFF_DOWNCOUNT);
    fexit.reg_cache.PrepareReturn(this, 1);
    LoadUnsignedImmediate(fexit.reg_cache.GetReturnRegister(0), 0);
    RestoreRegistersReturn(fexit.reg_cache.saved_regs);
  }

  for (auto& pair : exits)
  {
    if (!(pair.second.flags & FASTMEM_BAIL))
    {
      SetBranchTarget(pair.first);
    }
    else
    {
      fault_handlers.emplace_back(pair.first, this->instructions.size());
    }
    WriteExit(pair.second);
  }
}

void PPC64BaselineCompiler::WriteExit(const Exit& jump)
{
  RegisterCache rc = jump.reg_cache;
  GPR ppcs = rc.GetPPCState();
  GPR scratch = rc.GetScratch(this);
  if (jump.flags & (RAISE_FPU_EXCEPTION | RAISE_ALIGNMENT_EXCEPTION))
  {
    u16 exc;
    if (jump.flags & RAISE_FPU_EXCEPTION)
    {
      exc = u16(EXCEPTION_FPU_UNAVAILABLE);
    }
    else
    {
      ASSERT(jump.flags & RAISE_ALIGNMENT_EXCEPTION);
      exc = u16(EXCEPTION_ALIGNMENT);
      STW(static_cast<GPR>(jump.link_address), ppcs, SPROffset(SPR_DAR));
    }
    LWZ(scratch, ppcs, OFF_EXCEPTIONS);
    ORI(scratch, scratch, exc);
    STW(scratch, ppcs, OFF_EXCEPTIONS);
  }
  if (jump.flags & SKIP)
  {
    GPR target = rc.PrepareCall(this, 0);
    // ppcs is invalidated by PrepareCall
    ppcs = rc.GetPPCState();
    // call CoreTiming::Idle
    LD(target, rc.GetToC(), s16(s32(offsetof(TableOfContents, idle)) - 0x4000));
    rc.BindCall(this);
    rc.PerformCall(this, 0);
  }
  scratch = rc.GetScratch(this);
  if (jump.flags & JUMPSPR)
  {
    LWZ(scratch, ppcs, SPROffset(jump.address));
  }
  else
  {
    LoadUnsignedImmediate(scratch, jump.address);
  }
  STW(scratch, ppcs, OFF_PC);
  if (jump.flags & EXCEPTION)
  {
    ADDI(scratch, scratch, 4);
  }
  STW(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, npc)));
  if (jump.flags & LINK)
  {
    LoadUnsignedImmediate(scratch, jump.link_address);
    STW(scratch, ppcs, SPROffset(SPR_LR));
  }
  // decrement *after* skip – important for timing equivalency to Generic
  LWZ(scratch, ppcs, OFF_DOWNCOUNT);
  ADDI(scratch, scratch, -s16(jump.downcount));
  STW(scratch, ppcs, OFF_DOWNCOUNT);
  rc.PrepareReturn(this, 1);
  LoadUnsignedImmediate(rc.GetReturnRegister(0),
                        jump.flags & FASTMEM_BAIL ?
                            (JitTieredGeneric::BLOCK_OVERRUN | JitTieredGeneric::REPORT_BAIL) :
                            0);
  RestoreRegistersReturn(jump.reg_cache.saved_regs);
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
  u32 gprs_to_invalidate = 0;
  if (opinfo.flags & FL_OUT_D)
  {
    gprs_to_invalidate |= 1 << inst.RD;
  }
  if (opinfo.flags & FL_OUT_A)
  {
    gprs_to_invalidate |= 1 << inst.RA;
  }
  // special case is lmw, which uses more guest GPRs than its encoding lets on
  reg_cache.ReduceGuestRegisters(this, 0xffffffff,
                                 inst.OPCD != 46 ? gprs_to_invalidate : 0xffffffff);
  GPR ppcs = reg_cache.GetPPCState();
  GPR scratch = reg_cache.GetScratch(this);
  // set PC + NPC
  LoadUnsignedImmediate(scratch, address);
  STW(scratch, ppcs, OFF_PC);
  ADDI(scratch, scratch, 4);
  STW(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, npc)));
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

  GPR target = reg_cache.PrepareCall(this, 1);
  // ppcs was invalidated
  ppcs = reg_cache.GetPPCState();
  GPR toc = reg_cache.GetToC();
  LD(target, toc,
     s16(s32(offsetof(TableOfContents, fallback_table) + 8 * fallback_index) - 0x4000));
  reg_cache.BindCall(this);
  LoadUnsignedImmediate(reg_cache.GetArgumentRegister(0), inst.hex);
  reg_cache.PerformCall(this, 0);

  // scratch was invalidated
  scratch = reg_cache.GetScratch(this);
  // check for exceptions
  LWZ(scratch, ppcs, OFF_EXCEPTIONS);
  ANDI_Rc(scratch, scratch, u16(JitTieredGeneric::EXCEPTION_SYNC));
  // allocate scratch register before branch, so we can use the same exit block
  GPR scratch2 = reg_cache.GetScratch(this);
  auto exc_exit = BC(BR_FALSE, CR0 + EQ);
  // load NPC into scratch and return if it's ≠ PC + 4
  LWZ(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, npc)));
  XORIS(scratch2, scratch, u16((address + 4) >> 16));
  CMPLI(CR0, CMP_WORD, scratch2, u16((address + 4) & 0xffff));
  auto jump_exit = BC(BR_FALSE, CR0 + EQ);
  fallback_exits.push_back({reg_cache, scratch, jump_exit, exc_exit, downcount});
}

void PPC64BaselineCompiler::BCX(UGeckoInstruction inst, GekkoOPInfo& opinfo)
{
  if ((inst.BO & (BO_DONT_CHECK_CONDITION | BO_DONT_DECREMENT_FLAG)) == 0)
  {
    WARN_LOG(DYNA_REC, "CR+CTR branch %08x @ %08x", inst.hex, address);
  }
  bool inverted = false;
  u32 bit = 0;
  const GPR ppcs = reg_cache.GetPPCState();
  if (!(inst.BO & BO_DONT_CHECK_CONDITION))
  {
    const bool branch_on_true = inst.BO & BO_BRANCH_IF_TRUE;
    const GPR scratch = reg_cache.GetScratch(this);
    LD(scratch, ppcs, s16(offsetof(PowerPC::PowerPCState, cr_val) + 8 * (inst.BI / 4)));
    switch (inst.BI % 4)
    {
    case LT:
      INFO_LOG(DYNA_REC, "LT branch @ %08x", address);
      RLDICL(scratch, scratch, 2, 63, RC);
      inverted = branch_on_true;
      bit = CR0 + EQ;
      break;
    case GT:
      INFO_LOG(DYNA_REC, "GT branch @ %08x", address);
      CMPI(CR0, CMP_DWORD, scratch, 0);
      inverted = !branch_on_true;
      bit = CR0 + GT;
      break;
    case EQ:
      INFO_LOG(DYNA_REC, "EQ branch @ %08x", address);
      CMPLI(CR0, CMP_WORD, scratch, 0);
      inverted = !branch_on_true;
      bit = CR0 + EQ;
      break;
    case SO:
      INFO_LOG(DYNA_REC, "SO branch @ %08x", address);
      TW();
      RLDICL(scratch, scratch, 3, 63, RC);
      inverted = !branch_on_true;
      bit = CR0 + EQ;
    }
    reg_cache.FreeGPR(scratch);
  }
  if (!(inst.BO & BO_DONT_DECREMENT_FLAG))
  {
    const GPR scratch = reg_cache.GetScratch(this);
    LWZ(scratch, ppcs, SPROffset(SPR_CTR));
    ADDI(scratch, scratch, -1);
    STW(scratch, ppcs, SPROffset(SPR_CTR));
    CMPLI(CR1, CMP_WORD, scratch, 0);
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
    reg_cache.FreeGPR(scratch);
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
    exits.emplace_back(branch,
                       Exit{reg_cache, target, downcount, inst.LK ? LINK : JUMP, address + 4});
  }
  else
  {
    u32 reg = inst.SUBOP10 == 16 ? SPR_LR : SPR_CTR;
    exits.emplace_back(
        branch, Exit{reg_cache, reg, downcount, JUMPSPR | (inst.LK ? LINK : JUMP), address + 4});
  }
}

void PPC64BaselineCompiler::LoadStore(UGeckoInstruction inst, GekkoOPInfo& opinfo,
                                      const std::vector<u32>& bails)
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

  const bool use_slowmem = !bails.empty();

  if (use_slowmem)
  {
    WARN_LOG(DYNA_REC, "compiling slowmem access @ %08x", address);
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
      ASSERT(false);
    }
    const GPR target = reg_cache.PrepareCall(this, is_store ? 2 : 1);
    const GPR toc = reg_cache.GetToC();
    LD(target, toc, offset);
    reg_cache.BindCall(this);
  }

  // TODO: allocate this register more smartly to eliminate moves
  const GPR addr_reg = reg_cache.GetScratch(this, is_update ? LOCKED : SCRATCH);
  if (inst.RA != 0)
  {
    // calculate address
    GPR base = reg_cache.GetGPR(this, DIRTY_R + inst.RA);
    if (is_indexed)
    {
      GPR index = reg_cache.GetGPR(this, DIRTY_R + inst.RB);
      ADD(addr_reg, base, index);
    }
    else
    {
      ADDI(addr_reg, base, inst.SIMM_16);
    }
    RLDICL(addr_reg, addr_reg, 0, 32);
  }
  else
  {
    ASSERT(!is_update);
    if (is_indexed)
    {
      MoveReg(addr_reg, reg_cache.GetGPR(this, ZEXT_R + inst.RB));
    }
    else
    {
      LoadUnsignedImmediate(addr_reg, Common::BitCast<u32>(s32(inst.SIMM_16)));
    }
  }
  GPR rst = R0;
  if (!use_slowmem)
  {
    if (is_store)
    {
      rst = reg_cache.GetGPR(this, DIRTY_R + inst.RD);
    }
    else
    {
      rst = reg_cache.GetScratch(this);
    }
    GPR base = reg_cache.GetMemoryBase(this);

    exits.emplace_back(this->instructions.size(),
                       Exit{reg_cache, address, downcount, FASTMEM_BAIL, 0});

    switch (op >> 1)
    {
    case 0:
      LWBRX(rst, base, addr_reg);
      break;
    case 1:
      LBZX(rst, base, addr_reg);
      break;
    case 2:
      STWBRX(rst, base, addr_reg);
      break;
    case 3:
      STBX(rst, base, addr_reg);
      break;
    case 4:
      LHBRX(rst, base, addr_reg);
      break;
    case 5:
      LHBRX(rst, base, addr_reg);
      RLWINM(rst, rst, 16, 0, 15);
      EXTSW(rst, rst);
      SRAWI(rst, rst, 16);
      break;
    case 6:
      STHBRX(rst, base, addr_reg);
      break;
    default:
      ERROR_LOG(DYNA_REC, "unexpected opcode %08x @ %08x", inst.hex, address);
      ASSERT(false);
    }
  }
  else
  {
    GPR arg1 = reg_cache.GetArgumentRegister(0);
    if (!is_store)
    {
      MoveReg(arg1, addr_reg);
    }
    else
    {
      MoveReg(arg1, reg_cache.GetGPR(this, DIRTY_R + inst.RD));
      // make sure it's properly zero-extended
      if ((op >> 1) == 3)
      {
        RLDICL(arg1, arg1, 0, 56);
      }
      else if ((op >> 1) == 6)
      {
        RLDICL(arg1, arg1, 0, 48);
      }
      else if ((op >> 1) == 2)
      {
        RLDICL(arg1, arg1, 0, 32);
      }
      MoveReg(reg_cache.GetArgumentRegister(1), addr_reg);
    }
    reg_cache.PerformCall(this, is_store ? 0 : 1);

    const GPR scratch = reg_cache.GetScratch(this);
    const GPR ppcs = reg_cache.GetPPCState();
    // check for exceptions
    LWZ(scratch, ppcs, OFF_EXCEPTIONS);
    ANDI_Rc(scratch, scratch, u16(JitTieredGeneric::EXCEPTION_SYNC));
    exits.emplace_back(BC(BR_FALSE, CR0 + EQ), Exit{reg_cache, address, downcount, EXCEPTION, 0});
    if (!is_store)
    {
      rst = reg_cache.GetReturnRegister(0);
    }
  }
  // maintain GPR file
  if (is_update)
  {
    ASSERT(is_store || inst.RA != inst.RD);
    reg_cache.BindGPR(addr_reg, ZEXT_R + inst.RA);
  }
  if (!is_store)
  {
    reg_cache.BindGPR(rst, ((op >> 1) == 5 ? SEXT_R : ZEXT_R) + inst.RD);
  }
}
