// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/PPC64RegCache.h"

#include "Common/Assert.h"
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
  reg_state[5] = FREE;
  reg_state[30] = TOC_PTR;
}

void RegisterCache::SetCR0(PPCEmitter* emit, GPR host_gpr)
{
  if (HoldsGuestRegister(host_gpr) && !IsSignExtended(host_gpr))
  {
    emit->EXTSW(host_gpr, host_gpr);
    BindGPR(host_gpr, SEXT_R + GetGuestRegister(host_gpr));
  }
  // FIXME: implement SO emulation?
  emit->STD(host_gpr, GetGPR(emit, PPCSTATE_PTR), s16(offsetof(PowerPC::PowerPCState, cr_val)));
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
  // can only bind to guest registers
  ASSERT((specifier & 96) != 0);
  u8 reg = specifier & 31;
  // catch (some) invalidated registers
  ASSERT(reg_state[host_gpr] == SCRATCH ||
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
  u8 want_state = specifier & 96;
  if (want_state == 0)
  {
    // search for GPR
    u8 reg = specifier & 31;
    bool found_vacancy = false;
    bool found_clean = false;
    u8 spot = 0;
    for (u8 i = 0; i < 32; ++i)
    {
      const u16 state = reg_state[i];
      if ((state & 31) == reg)
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
        reg_state[i] |= (specifier & FLAG_GUEST_UNSAVED) | FLAG_IN_USE;
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
    const GPR ppcs = GetGPR(emit, PPCSTATE_PTR);
    if (found_vacancy || found_clean)
    {
      const GPR res = static_cast<GPR>(spot);
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
  else if (specifier == SCRATCH || specifier == LOCKED)
  {
    const u8 start_search = specifier == SCRATCH ? 2 : 14;
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
    const GPR ppcs = GetGPR(emit, PPCSTATE_PTR);
    if (found_vacancy || found_clean)
    {
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
        reg_state[host_reg] = specifier;
        return host_reg;
      }
    }
    ERROR_LOG(DYNA_REC, "No free register to allocate scratch register");
    ASSERT(false);
    return PPCEmitter::R0;
  }
  else
  {
    for (u8 i = 0; i < 32; ++i)
    {
      if (reg_state[i] == specifier)
      {
        return static_cast<GPR>(i);
      }
    }
    ERROR_LOG(DYNA_REC, "No register for specifier %u was found", u32(specifier));
    ASSERT(false);
    return PPCEmitter::R0;
  }
}

void RegisterCache::FlushHostRegister(PPCEmitter* emit, GPR host_gpr)
{
  const GPR ppcs = GetGPR(emit, PPCSTATE_PTR);
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
    reg_state[i] = ABI_FREE;
  }
  return PPCEmitter::R12;
}

void RegisterCache::BindCall(PPCEmitter* emit)
{
  emit->MTSPR(PPCEmitter::SPR_CTR, PPCEmitter::R12);
}

void RegisterCache::PerformCall(PPCEmitter* emit, u8 num_return_gprs)
{
  ASSERT(num_return_gprs < 10);
  emit->BCCTR();
  for (u8 i = 3; i < 3 + num_return_gprs; ++i)
  {
    ASSERT(reg_state[i] == SCRATCH || reg_state[i] == ABI_FREE);
    reg_state[i] = ABI;
  }
  for (u8 i = 3 + num_return_gprs; i < 13; ++i)
  {
    ASSERT(reg_state[i] == SCRATCH || reg_state[i] == ABI_FREE);
    reg_state[i] = FREE;
  }
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
