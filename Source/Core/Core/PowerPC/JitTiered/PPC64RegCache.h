// Copyright 2019 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/PPCEmitter.h"
#include "Core/PowerPC/PowerPC.h"

class PPC64BaselineCompiler;

namespace PPC64RegCache
{
using GPR = PPCEmitter::GPR;
using FPR = PPCEmitter::FPR;
/// the least significant 5 bit specify which guest register (if applicable) or what other state the
/// register is in. the next two bits decide what state the guest register is in (0b00 ↦ not a guest
/// register, 0b01 ↦ dirty, 0b10 ↦ zero extended, 0b11 ↦ sign extended) upper bits are flags
enum : u16
{
  FREE = 0,
  RESERVED = 1,
  TOC_PTR = 2,
  PPCSTATE_PTR = 3,
  SCRATCH = 4,
  /// some non-guest-register value, is in non-volatile host register and is not invalidated
  LOCKED = 5,
  /// argument for a prepared (but not performed) call, or return value of a recently-performed call
  /// not available for register caching, but can be claimed using GetArgumentRegister and
  /// GetReturnRegister
  ABI = 6,
  MEMORY_BASE = 7,
  DIRTY_R = 32,
  ZEXT_R = 64,
  SEXT_R = 96,
  /// means the register has not been written to ppcState yet.
  /// passing this to GetGPR signals that the guest register will be written there without fail
  FLAG_GUEST_UNSAVED = 128,
  /// this is used to avoid overwriting registers that are used by the current intruction
  FLAG_IN_USE = 256,
};

enum : u16
{
  FPR_FREE = 0,
  FPR_RESERVED = 1,
  FPR_SCRATCH = 2,
  SPLAT_F = 32,
  PS0_F = 64,
  PS1_F = 96,
  MASK_PS_MEMBER = 96,
  FLAG_FPR_UNSAVED = 1 << 14,
  FLAG_FPR_IN_USE = 1 << 15,
};

struct RegisterCache
{
  /// state of host registers at this point
  std::array<u16, 32> reg_state{};
  bool guest_fpscr = false;
  std::array<u16, 32> fpr_state{};

  RegisterCache()
  {
    // register 0 is special in addi and load/store, so better not use it for now
    reg_state[0] = RESERVED;
    // register 1 is always the stack pointer
    reg_state[1] = RESERVED;
    // register 2 is supposed to be the TOC pointer. we use it for the current memory base, if code
    // does any fastmem accesses
    reg_state[2] = RESERVED;
    // register 13 is the thread pointer per ABI
    reg_state[13] = RESERVED;
    // register 6 is the fourth argument, starts out as the ToC pointer
    reg_state[30] = TOC_PTR;
    // register 5 is the third argument, starts out as the ppcState pointer
    reg_state[31] = PPCSTATE_PTR;
    // FPRs 0–2 are used as temporaries in float routines/sequences, so keep them reserved
    fpr_state[0] = fpr_state[1] = fpr_state[2] = FPR_RESERVED;
    // we don't (want to) use non-volatile FPRs
    for (u8 i = 14; i < 32; ++i)
    {
      fpr_state[i] = FPR_RESERVED;
    }
  }
  bool HoldsGuestRegister(GPR host_gpr) { return (reg_state[host_gpr] & 96) != 0; }
  bool IsSignExtended(GPR host_gpr) { return (reg_state[host_gpr] & 96) == SEXT_R; }
  bool IsZeroExtended(GPR host_gpr) { return (reg_state[host_gpr] & 96) == ZEXT_R; }
  GPR GetGuestRegister(GPR host_gpr) { return static_cast<GPR>(reg_state[host_gpr] & 31); }
  /// invalidates all register references that are not LOCKED, TOC_PTR or PPCSTATE_PTR
  void ReleaseRegisters()
  {
    for (u8 i = 0; i < 32; ++i)
    {
      if (reg_state[i] != LOCKED)
      {
        reg_state[i] &= ~FLAG_IN_USE;
      }
      if (reg_state[i] == SCRATCH)
      {
        reg_state[i] = FREE;
      }
      fpr_state[i] &= ~FLAG_FPR_IN_USE;
      if (fpr_state[i] == FPR_SCRATCH)
      {
        fpr_state[i] = FREE;
      }
    }
  }
  void FreeGPR(GPR host_gpr) { reg_state[host_gpr] = FREE; }
  GPR GetArgumentRegister(u8 index);
  GPR GetReturnRegister(u8 index);
  void BindGPR(GPR host_gpr, u16 specifier);
  void BindFPR(FPR host_fpr, u16 specifier);
  void InvalidateAllRegisters();

  void FlushHostRegister(PPCEmitter* emit, GPR gpr);
  void FlushAllRegisters(PPC64BaselineCompiler* comp);
  void ReduceGuestRegisters(PPCEmitter* emit, u32 gprs_to_flush, u32 gprs_to_invalidate,
                            u32 fprs_to_flush, u32 fprs_to_invalidate);
  void RestoreStandardState(PPC64BaselineCompiler* comp);

  GPR GetGPR(PPCEmitter* emit, u16 specifier);
  GPR GetScratch(PPCEmitter* emit, u16 specifier = SCRATCH);
  GPR GetPPCState();
  GPR GetToC();
  GPR GetMemoryBase(PPCEmitter* emit);

  FPR GetFPR(PPCEmitter* emit, u16 specifier);
  FPR GetFPRScratch(PPCEmitter* emit);
  void FlushHostFPR(PPCEmitter* emit, FPR fpr);

  void GuestFPSCR(PPC64BaselineCompiler* comp);
  void HostFPSCR(PPC64BaselineCompiler* comp);

  /// invalidates all non-LOCKED register references
  GPR PrepareCall(PPCEmitter* emit, u8 num_parameters);
  void BindCall(PPCEmitter* emit);
  /// invalidates all register references that are not LOCKED, TOC_PTR or PPCSTATE_PTR
  void PerformCall(PPC64BaselineCompiler* comp, u8 num_return_gprs);
  /// invalidates all register references that are not LOCKED, TOC_PTR or PPCSTATE_PTR
  void PrepareReturn(PPC64BaselineCompiler* comp, u8 num_return_values);

  void SetCR0(PPCEmitter* emit, GPR host_gpr);
};

}  // namespace PPC64RegCache
