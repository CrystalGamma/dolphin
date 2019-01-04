// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64RegCache.h"

#include "Common/Assert.h"
#include "Common/Logging/Log.h"
#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

namespace PPC64RegCache
{
static constexpr s16 GPROffset(u32 i)
{
  return s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * i);
}

void RegisterCache::EstablishStackFrame(PPC64BaselineCompiler* comp)
{
  ASSERT(saved_regs == 0);
  // allocate stack frame, save caller registers
  comp->MFSPR(PPCEmitter::R0, PPCEmitter::SPR_LR);
  comp->STD(PPCEmitter::R0, PPCEmitter::R1, 16);
  saved_regs = 3;
  comp->relocations.push_back(
      comp->B(PPCEmitter::BR_LINK, comp->offsets.save_gprs + (18 - saved_regs) * 4));
  comp->STD(PPCEmitter::R1, PPCEmitter::R1, -32 - 8 * saved_regs, PPCEmitter::UPDATE);
  // copy our variables
  comp->MoveReg(PPCEmitter::R31, PPCEmitter::R5);
  reg_state[5] = FREE;
  reg_state[31] = PPCSTATE_PTR;
  comp->ADDI(PPCEmitter::R30, PPCEmitter::R6, 0x4000);
  reg_state[6] = FREE;
  reg_state[30] = TOC_PTR;

  reg_state[29] = FREE;
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
    if (reg_state[i] != RESERVED && reg_state[i] != HOST_UNSAVED && reg_state[i] != TOC_PTR &&
        reg_state[i] != PPCSTATE_PTR)
    {
      reg_state[i] = FREE;
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
  const u8 start_search = specifier == SCRATCH ? 0 : 14;
  const u8 end_search = specifier == SCRATCH ? 13 : 32;
  bool found_vacancy = false;
  bool found_clean = false;
  u8 spot = 0;
  for (u8 i = start_search; i < end_search; ++i)
  {
    const u16 state = reg_state[i];
    if (state == FREE || state == ABI_FREE)
    {
      found_vacancy = true;
      spot = i;
      break;
    }
    else if (!found_clean && (state & 96) != 0 && !(state & (FLAG_GUEST_UNSAVED | FLAG_IN_USE)))
    {
      found_clean = true;
      spot = i;
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
  for (u8 i = start_search; i < end_search; ++i)
  {
    const GPR host_reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(host_reg) && !(reg_state[host_reg] & FLAG_IN_USE))
    {
      emit->STW(host_reg, ppcs, GPROffset(GetGuestRegister(host_reg)));
      INFO_LOG(DYNA_REC, "creating scratch in %u (spill)", u32(host_reg));
      reg_state[host_reg] = specifier;
      return host_reg;
    }
  }
  ERROR_LOG(DYNA_REC, "No free register to allocate scratch register");
  ASSERT(false);
  return PPCEmitter::R0;
}

GPR RegisterCache::GetPPCState()
{
  for (u8 i = 0; i < 32; ++i)
  {
    if (reg_state[i] == PPCSTATE_PTR)
    {
      return static_cast<GPR>(i);
    }
  }
  ERROR_LOG(DYNA_REC, "ppcState went missing!");
  ASSERT(false);
  return PPCEmitter::R0;
}

GPR RegisterCache::GetToC()
{
  for (u8 i = 0; i < 32; ++i)
  {
    if (reg_state[i] == TOC_PTR)
    {
      return static_cast<GPR>(i);
    }
  }
  ERROR_LOG(DYNA_REC, "ToC went missing!");
  ASSERT(false);
  return PPCEmitter::R0;
}

void RegisterCache::FlushHostRegister(PPCEmitter* emit, GPR host_gpr)
{
  const GPR ppcs = GetPPCState();
  ASSERT(HoldsGuestRegister(host_gpr));
  if (reg_state[host_gpr] & FLAG_GUEST_UNSAVED)
  {
    emit->STW(host_gpr, ppcs, GPROffset(GetGuestRegister(host_gpr)));
    reg_state[host_gpr] &= ~FLAG_GUEST_UNSAVED;
  }
}

void RegisterCache::FlushAllRegisters(PPCEmitter* emit)
{
  for (u8 i = 0; i < 32; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(emit, reg);
    }
  }
}

GPR RegisterCache::PrepareCall(PPCEmitter* emit, u8 num_parameters)
{
  ASSERT(num_parameters <= 10);
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

void RegisterCache::PerformCall(PPCEmitter* emit, u8 num_return_gprs)
{
  ASSERT(num_return_gprs < 10);
  for (u8 i = 0; i < 13; ++i)
  {
    const GPR reg = static_cast<GPR>(i);
    if (HoldsGuestRegister(reg))
    {
      FlushHostRegister(emit, reg);
    }
    if (reg_state[i] != RESERVED)
    {
      reg_state[i] = i >= 3 && i < 3 + num_return_gprs ? ABI : FREE;
    }
  }
  emit->BCCTR();
}

void RegisterCache::PrepareReturn(PPCEmitter* emit, u8 num_return_gprs)
{
  FlushAllRegisters(emit);
  for (u8 i = 3; i < 3 + num_return_gprs; ++i)
  {
    reg_state[i] = ABI;
  }
}

}  // namespace PPC64RegCache
