// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64RegCache.h"

#include "Common/Assert.h"
#include "Common/Logging/Log.h"
#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

#define OFF_PS0(reg) s16(offsetof(PowerPC::PowerPCState, ps) + 16 * (reg))
#define OFF_PS1(reg) s16(offsetof(PowerPC::PowerPCState, ps) + 16 * (reg) + 8)

namespace PPC64RegCache
{
static constexpr s16 GPROffset(u32 i)
{
  return s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * i);
}

void RegisterCache::SetCR0(PPCEmitter* emit, GPR host_gpr)
{
  if (HoldsGuestRegister(host_gpr) && !IsSignExtended(host_gpr))
  {
    emit->EXTSW(host_gpr, host_gpr);
    BindGPR(host_gpr, SEXT_R + GetGuestRegister(host_gpr));
  }
  // FIXME: implement SO emulation?
  emit->STD(host_gpr, GetPPCState(), s16(offsetof(PowerPC::PowerPCState, cr_val)));
}

GPR RegisterCache::GetArgumentRegister(u8 index)
{
  ASSERT(index < 10 && reg_state[index + 3] == ABI);
  reg_state[index + 3] = SCRATCH;
  return static_cast<GPR>(index + 3);
}

GPR RegisterCache::GetReturnRegister(u8 index)
{
  ASSERT(index < 10 && reg_state[index + 3] == ABI);
  reg_state[index + 3] = SCRATCH;
  return static_cast<GPR>(index + 3);
}

void RegisterCache::BindGPR(GPR host_gpr, u16 specifier)
{
  INFO_LOG(DYNA_REC, "binding %u (which is 0x%x) to 0x%x", u32(host_gpr), u32(reg_state[host_gpr]),
           u32(specifier));
  // can only bind to guest registers
  ASSERT((specifier & 96) != 0);
  u8 reg = specifier & 31;
  // catch (some) invalidated registers
  ASSERT(reg_state[host_gpr] == SCRATCH || reg_state[host_gpr] == LOCKED ||
         (HoldsGuestRegister(host_gpr) && GetGuestRegister(host_gpr) == reg));
  // invalidate any other values for the same guest register
  for (u16& rs : reg_state)
  {
    if ((rs & 96) != 0 && (rs & 31) == reg)
    {
      rs = FREE;
    }
  }
  reg_state[host_gpr] = specifier | FLAG_GUEST_UNSAVED | FLAG_IN_USE;
}

void RegisterCache::InvalidateAllRegisters()
{
  for (u8 i = 0; i < 32; ++i)
  {
    const u16 prev_state = reg_state[i];
    ASSERT(!(prev_state & FLAG_GUEST_UNSAVED));
    if (reg_state[i] != RESERVED && reg_state[i] != TOC_PTR && reg_state[i] != PPCSTATE_PTR)
    {
      reg_state[i] = FREE;
    }
    ASSERT(!(fpr_state[i] & FLAG_FPR_UNSAVED));
    if (fpr_state[i] != FPR_RESERVED)
    {
      fpr_state[i] = FPR_FREE;
    }
  }
}

GPR RegisterCache::GetGPR(PPCEmitter* emit, u16 specifier)
{
  INFO_LOG(DYNA_REC, "getting 0x%x", u32(specifier));
  ASSERT((specifier & ~FLAG_GUEST_UNSAVED) >= DIRTY_R &&
         (specifier & ~FLAG_GUEST_UNSAVED) < SEXT_R + 32);
  const u8 want_state = specifier & 96;
  u8 reg = specifier & 31;
  bool found_vacancy = false;
  bool found_clean = false;
  u8 spot = 0;
  for (u8 i = 0; i < 32; ++i)
  {
    const u16 state = reg_state[i];
    if ((state & 96) != 0 && (state & 31) == reg)
    {
      GPR res = static_cast<GPR>(i);
      if (want_state != DIRTY_R && want_state != (state & 96))
      {
        if (want_state == ZEXT_R)
        {
          emit->RLDICL(res, res, 0, 32);
        }
        else
        {
          ASSERT(want_state == SEXT_R);
          emit->EXTSW(res, res);
        }
      }
      // FIXME: the |= looks wrong
      reg_state[i] |= specifier | FLAG_IN_USE | (reg_state[i] & FLAG_GUEST_UNSAVED);
      INFO_LOG(DYNA_REC, "reusing 0x%x in %u", u32(reg_state[i]), u32(i));
      return res;
    }
    if (!found_vacancy)
    {
      if (state == FREE)
      {
        found_vacancy = true;
        spot = i;
      }
      else if (!found_clean && (state & 96) != 0 && !(state & (FLAG_GUEST_UNSAVED | FLAG_IN_USE)))
      {
        found_clean = true;
        spot = i;
      }
    }
  }
  // ppcState pointer is not a guest register, so this should not recurse endlessly
  const GPR ppcs = GetPPCState();
  if (found_vacancy || found_clean)
  {
    const GPR res = static_cast<GPR>(spot);
    INFO_LOG(DYNA_REC, "loading 0x%x into %u (no spill)", u32(specifier), u32(spot));
    emit->LWZ(res, ppcs, GPROffset(reg));
    if (want_state == SEXT_R)
    {
      emit->EXTSW(res, res);
    }
    reg_state[spot] = specifier | FLAG_IN_USE;
    return res;
  }
  // evict guest register
  for (u8 i = 0; i < 32; ++i)
  {
    const GPR host_reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(host_reg) && !(reg_state[host_reg] & FLAG_IN_USE))
    {
      emit->STW(host_reg, ppcs, GPROffset(GetGuestRegister(host_reg)));
      INFO_LOG(DYNA_REC, "loading 0x%x into %u (spill)", u32(specifier), u32(host_reg));
      emit->LWZ(host_reg, ppcs, GPROffset(reg));
      if (want_state == SEXT_R)
      {
        emit->EXTSW(host_reg, host_reg);
      }
      reg_state[host_reg] = specifier | FLAG_IN_USE;
      return host_reg;
    }
  }
  ERROR_LOG(DYNA_REC, "No free register to load guest register");
  ASSERT(false);
  return PPCEmitter::R0;
}

GPR RegisterCache::GetScratch(PPCEmitter* emit, u16 specifier)
{
  ASSERT(specifier == SCRATCH || specifier == LOCKED);
  const s8 first_search = 31;
  const s8 last_search = specifier == SCRATCH ? 0 : 14;
  bool found_vacancy = false;
  bool found_clean = false;
  u8 spot = 0;
  for (s8 i = first_search; i >= last_search; --i)
  {
    const u16 state = reg_state[i];
    if (state == FREE)
    {
      found_vacancy = true;
      spot = u8(i);
      break;
    }
    else if (!found_clean && (state & 96) != 0 && !(state & (FLAG_GUEST_UNSAVED | FLAG_IN_USE)))
    {
      found_clean = true;
      spot = u8(i);
    }
  }
  // ppcState pointer is not a guest register, so this should not recurse endlessly
  const GPR ppcs = GetPPCState();
  if (found_vacancy || found_clean)
  {
    INFO_LOG(DYNA_REC, "creating scratch in %u (no spill)", u32(spot));
    reg_state[spot] = specifier;
    return static_cast<GPR>(spot);
  }
  // evict guest register
  for (u8 i = first_search; i >= last_search; --i)
  {
    const GPR host_reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(host_reg) && !(reg_state[host_reg] & FLAG_IN_USE))
    {
      emit->STW(host_reg, ppcs, GPROffset(GetGuestRegister(host_reg)));
      WARN_LOG(DYNA_REC, "creating scratch in %u (spill)", u32(host_reg));
      reg_state[host_reg] = specifier;
      return host_reg;
    }
  }
  ERROR_LOG(DYNA_REC, "No free register to allocate scratch register");
  ASSERT(false);
  return PPCEmitter::R0;
}

GPR RegisterCache::GetMemoryBase(PPCEmitter* emit)
{
  for (u8 i = 0; i < 32; ++i)
  {
    if (reg_state[i] == MEMORY_BASE)
    {
      return static_cast<GPR>(i);
    }
  }
  ASSERT(reg_state[2] == RESERVED);
  const GPR base = PPCEmitter::R2;
  emit->LWZ(base, GetPPCState(), s16(offsetof(PowerPC::PowerPCState, msr)));
  // make it so that we have 0 if address translation disabled and 8 if enabled
  emit->RLWINM(base, base, 31, 28, 28);
  // add offset to physical_base
  emit->ADDI(base, base,
             s16(s32(offsetof(PPC64BaselineCompiler::TableOfContents, physical_base)) - 0x4000));
  // use indexed load to fetch the base pointer
  emit->LDX(base, base, GetToC());
  reg_state[2] = MEMORY_BASE;
  return base;
}

GPR RegisterCache::GetPPCState()
{
  ASSERT(reg_state[31] == PPCSTATE_PTR);
  return PPCEmitter::R31;
}

GPR RegisterCache::GetToC()
{
  ASSERT(reg_state[30] == TOC_PTR);
  return PPCEmitter::R30;
}

void RegisterCache::FlushHostRegister(PPCEmitter* emit, GPR host_gpr)
{
  const GPR ppcs = GetPPCState();
  ASSERT(HoldsGuestRegister(host_gpr));
  if (reg_state[host_gpr] & FLAG_GUEST_UNSAVED)
  {
    emit->STW(host_gpr, ppcs, GPROffset(GetGuestRegister(host_gpr)));
    reg_state[host_gpr] &= ~FLAG_GUEST_UNSAVED;
    INFO_LOG(DYNA_REC, "spilling guest GPR %u", u32(host_gpr));
  }
}

void RegisterCache::FlushAllRegisters(PPC64BaselineCompiler* comp)
{
  HostFPSCR(comp);
  for (u8 i = 0; i < 32; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(comp, reg);
    }
    const FPR host_fpr = static_cast<FPR>(i);
    if (fpr_state[i] & MASK_PS_MEMBER)
    {
      FlushHostFPR(comp, host_fpr);
    }
  }
}

void RegisterCache::ReduceGuestRegisters(PPCEmitter* emit, u32 gprs_to_flush,
                                         u32 gprs_to_invalidate, u32 fprs_to_flush,
                                         u32 fprs_to_invalidate)
{
  for (u8 i = 0; i < 32; ++i)
  {
    GPR host_reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(host_reg))
    {
      u8 guest_reg = GetGuestRegister(host_reg);
      if (gprs_to_flush & (1 << guest_reg))
      {
        FlushHostRegister(emit, host_reg);
      }
      if (gprs_to_invalidate & (1 << guest_reg))
      {
        reg_state[i] = FREE;
      }
    }
    FPR host_fpr = static_cast<FPR>(i);
    if (fpr_state[i] & MASK_PS_MEMBER)
    {
      u8 guest_fpr = fpr_state[i] & 31;
      if (fprs_to_flush & (1 << guest_fpr))
      {
        FlushHostFPR(emit, host_fpr);
      }
      if (fprs_to_invalidate & (1 << guest_fpr))
      {
        fpr_state[i] = FPR_FREE;
      }
    }
  }
}

void RegisterCache::RestoreStandardState(PPC64BaselineCompiler* comp)
{
  ReduceGuestRegisters(static_cast<PPCEmitter*>(comp), 0xffffffff, 0xffffffff, 0xffffffff,
                       0xffffffff);
  reg_state[2] = RESERVED;
  HostFPSCR(comp);
}

GPR RegisterCache::PrepareCall(PPCEmitter* emit, u8 num_parameters)
{
  ASSERT(num_parameters <= 10);
  for (u8 i = 14; i < 32; ++i)
  {
    if (reg_state[i] == FREE ||
        (HoldsGuestRegister(static_cast<GPR>(i)) && !(reg_state[i] & FLAG_GUEST_UNSAVED)))
    {
      for (u8 j = 0; j < 13; ++j)
      {
        if (reg_state[j] & FLAG_GUEST_UNSAVED)
        {
          INFO_LOG(DYNA_REC, "saving guest register %u (was in %u as %x) to %u (which was %x)",
                   u32(GetGuestRegister(static_cast<GPR>(j))), u32(j), u32(reg_state[j]), u32(i),
                   u32(reg_state[i]));
          emit->MoveReg(static_cast<GPR>(i), static_cast<GPR>(j));
          reg_state[i] = reg_state[j];
          reg_state[j] = FREE;
          break;
        }
      }
    }
  }
  for (u8 i = 14; i < 32; ++i)
  {
    if (reg_state[i] == FREE)
    {
      for (u8 j = 0; j < 13; ++j)
      {
        if (HoldsGuestRegister(static_cast<GPR>(j)))
        {
          INFO_LOG(DYNA_REC, "saving guest register %u (was in %u as %x) to %u (which was %x)",
                   u32(GetGuestRegister(static_cast<GPR>(j))), u32(j), u32(reg_state[j]), u32(i),
                   u32(reg_state[i]));
          emit->MoveReg(static_cast<GPR>(i), static_cast<GPR>(j));
          reg_state[i] = reg_state[j];
          reg_state[j] = FREE;
          break;
        }
      }
    }
  }

  if (reg_state[2] == MEMORY_BASE)
  {
    reg_state[2] = RESERVED;
  }
  for (u8 i = 3; i < 3 + num_parameters; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(emit, reg);
    }
    reg_state[i] = ABI;
  }
  for (u8 i = 3 + num_parameters; i < 13; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(emit, reg);
    }
    else
    {
      reg_state[i] = FREE;
    }
  }
  for (u8 i = 0; i < 14; ++i)
  {
    const FPR fpr = static_cast<FPR>(i);
    if (fpr_state[i] & MASK_PS_MEMBER)
    {
      FlushHostFPR(emit, fpr);
    }
    else
    {
      fpr_state[i] = FPR_FREE;
    }
  }
  if (HoldsGuestRegister(PPCEmitter::R12))
  {
    FlushHostRegister(emit, PPCEmitter::R12);
  }
  reg_state[12] = SCRATCH;
  return PPCEmitter::R12;
}

void RegisterCache::BindCall(PPCEmitter* emit)
{
  emit->MTSPR(PPCEmitter::SPR_CTR, PPCEmitter::R12);
}

void RegisterCache::PerformCall(PPC64BaselineCompiler* comp, u8 num_return_gprs)
{
  ASSERT(num_return_gprs < 10);
  for (u8 i = 0; i < 13; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(comp, reg);
    }
    if (reg_state[i] != RESERVED)
    {
      reg_state[i] = i >= 3 && i < 3 + num_return_gprs ? ABI : FREE;
    }
    if (fpr_state[i] & MASK_PS_MEMBER)
    {
      FlushHostFPR(comp, static_cast<FPR>(i));
    }
    if (fpr_state[i] != FPR_RESERVED)
    {
      fpr_state[i] = FPR_FREE;
    }
  }
  HostFPSCR(comp);
  comp->BCCTR();
}

void RegisterCache::PrepareReturn(PPC64BaselineCompiler* comp, u8 num_return_gprs)
{
  FlushAllRegisters(comp);
  for (u8 i = 3; i < 3 + num_return_gprs; ++i)
  {
    reg_state[i] = ABI;
  }
}

void RegisterCache::GuestFPSCR(PPC64BaselineCompiler* comp)
{
  if (guest_fpscr)
  {
    return;
  }
  comp->relocations.push_back(comp->B(PPCEmitter::BR_LINK, comp->offsets.guest_fpscr));
  guest_fpscr = true;
}

void RegisterCache::HostFPSCR(PPC64BaselineCompiler* comp)
{
  if (!guest_fpscr)
  {
    return;
  }
  // this routine uses r0 and r2
  reg_state[2] = RESERVED;
  comp->relocations.push_back(comp->B(PPCEmitter::BR_LINK, comp->offsets.host_fpscr));
  guest_fpscr = false;
}

FPR RegisterCache::GetFPR(PPCEmitter* emit, u16 specifier)
{
  INFO_LOG(DYNA_REC, "getting FPR 0x%x", u32(specifier));
  ASSERT(specifier >= PS0_F && specifier < PS1_F + 32);
  const u8 want_member = specifier & 96;
  u8 reg = specifier & 31;
  bool found_vacancy = false;
  bool found_clean = false;
  u8 spot = 0;
  for (u8 i = 0; i < 32; ++i)
  {
    const u16 state = fpr_state[i];
    if ((state & 127) == specifier || (state & 127) == SPLAT_F + reg)
    {
      FPR res = static_cast<FPR>(i);
      fpr_state[i] |= FLAG_FPR_IN_USE;
      INFO_LOG(DYNA_REC, "reusing FPR 0x%x in %u", u32(fpr_state[i]), u32(i));
      return res;
    }
    if (!found_vacancy)
    {
      if (state == FPR_FREE)
      {
        found_vacancy = true;
        spot = i;
      }
      else if (!found_clean && (state & 96) != 0 && !(state & (FLAG_FPR_UNSAVED | FLAG_FPR_IN_USE)))
      {
        found_clean = true;
        spot = i;
      }
    }
  }
  const s16 offset = want_member == PS0_F ? OFF_PS0(reg) : OFF_PS1(reg);
  const GPR ppcs = GetPPCState();
  if (found_vacancy || found_clean)
  {
    const FPR res = static_cast<FPR>(spot);
    INFO_LOG(DYNA_REC, "loading FPR 0x%x into %u (no spill)", u32(specifier), u32(spot));
    emit->LFD(res, ppcs, offset);
    fpr_state[spot] = specifier | FLAG_FPR_IN_USE;
    return res;
  }
  // evict guest register
  for (u8 i = 0; i < 32; ++i)
  {
    const FPR host_fpr = static_cast<FPR>(i);
    if ((fpr_state[host_fpr] & MASK_PS_MEMBER) && !(fpr_state[host_fpr] & FLAG_FPR_IN_USE))
    {
      FlushHostFPR(emit, host_fpr);
      INFO_LOG(DYNA_REC, "loading FPR 0x%x into %u (spill)", u32(specifier), u32(host_fpr));
      emit->LFD(host_fpr, ppcs, offset);
      fpr_state[host_fpr] = specifier | FLAG_FPR_IN_USE;
      return host_fpr;
    }
  }
  ERROR_LOG(DYNA_REC, "No free FPR to load guest FPR");
  ASSERT(false);
  return PPCEmitter::F0;
}

FPR RegisterCache::GetFPRScratch(PPCEmitter* emit)
{
  const s8 first_search = 13;
  const s8 last_search = 3;
  bool found_vacancy = false;
  bool found_clean = false;
  u8 spot = 0;
  for (s8 i = first_search; i >= last_search; --i)
  {
    const u16 state = fpr_state[i];
    if (state == FPR_FREE)
    {
      found_vacancy = true;
      spot = u8(i);
      break;
    }
    else if (!found_clean && (state & 96) != 0 && !(state & (FLAG_FPR_UNSAVED | FLAG_FPR_IN_USE)))
    {
      found_clean = true;
      spot = u8(i);
    }
  }
  if (found_vacancy || found_clean)
  {
    INFO_LOG(DYNA_REC, "creating scratch FPR in %u (no spill)", u32(spot));
    fpr_state[spot] = FPR_SCRATCH;
    return static_cast<FPR>(spot);
  }
  // evict guest register
  for (u8 i = first_search; i >= last_search; --i)
  {
    const FPR host_fpr = static_cast<FPR>(i);
    if ((fpr_state[host_fpr] & MASK_PS_MEMBER) && !(fpr_state[host_fpr] & FLAG_FPR_IN_USE))
    {
      FlushHostFPR(emit, host_fpr);
      WARN_LOG(DYNA_REC, "creating scratch FPR in %u (spill)", u32(host_fpr));
      fpr_state[host_fpr] = FPR_SCRATCH;
      return host_fpr;
    }
  }
  ERROR_LOG(DYNA_REC, "No free register to allocate scratch FPR");
  ASSERT(false);
  return PPCEmitter::F0;
}

void RegisterCache::BindFPR(FPR host_fpr, u16 specifier)
{
  INFO_LOG(DYNA_REC, "binding FPR %u (which is 0x%x) to 0x%x", u32(host_fpr),
           u32(fpr_state[host_fpr]), u32(specifier));
  // can only bind to guest registers
  ASSERT((specifier & MASK_PS_MEMBER) != 0);
  u8 reg = specifier & 31;
  u16 member = specifier & MASK_PS_MEMBER;
  ASSERT(fpr_state[host_fpr] == FPR_SCRATCH);
  // invalidate any other values for the same member
  for (u16& rs : fpr_state)
  {
    if ((rs & 31) == reg)
    {
      if ((rs & 96) == (specifier & 96) || (member == SPLAT_F))
      {
        rs = FPR_FREE;
      }
      else if ((rs & 96) == SPLAT_F)
      {
        rs = (rs & ~MASK_PS_MEMBER) | (member == PS0_F ? PS1_F : PS0_F);
      }
    }
  }
  fpr_state[host_fpr] = specifier | FLAG_FPR_UNSAVED | FLAG_IN_USE;
}

void RegisterCache::FlushHostFPR(PPCEmitter* emit, FPR host_fpr)
{
  const GPR ppcs = GetPPCState();
  const u16 member = fpr_state[host_fpr] & MASK_PS_MEMBER;
  ASSERT(member != 0);
  if (fpr_state[host_fpr] & FLAG_FPR_UNSAVED)
  {
    if (member == SPLAT_F)
    {
      emit->STFD(host_fpr, ppcs, OFF_PS0(fpr_state[host_fpr] & 31));
      emit->STFD(host_fpr, ppcs, OFF_PS1(fpr_state[host_fpr] & 31));
    }
    else if (member == PS0_F)
    {
      emit->STFD(host_fpr, ppcs, OFF_PS0(fpr_state[host_fpr] & 31));
    }
    else
    {
      emit->STFD(host_fpr, ppcs, OFF_PS1(fpr_state[host_fpr] & 31));
    }
    fpr_state[host_fpr] &= ~FLAG_FPR_UNSAVED;
    INFO_LOG(DYNA_REC, "spilling guest FPR %u", u32(host_fpr));
  }
}

}  // namespace PPC64RegCache
