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
/// the least significant 5 bit specify which guest register (if applicable) or what other state the
/// register is in. the next two bits decide what state the guest register is in (0b00 ↦ not a guest
/// register, 0b01 ↦ dirty, 0b10 ↦ zero extended, 0b11 ↦ sign extended) upper bits are flags
enum : u16
{
  FREE = 0,
  HOST_UNSAVED = 1,
  RESERVED = 2,
  TOC_PTR = 3,
  PPCSTATE_PTR = 4,
  SCRATCH = 5,
  /// some non-guest-register value, is in non-volatile host register and is not invalidated
  LOCKED = 6,
  /// argument for a prepared (but not performed) call, or return value of a recently-performed call
  /// not available for register caching, but can be claimed using GetArgumentRegister and
  /// GetReturnRegister
  ABI = 7,
  /// volatile register during call preparation. available for scratch allocation, but not guest
  /// registers
  ABI_FREE = 8,
  DIRTY_R = 32,
  ZEXT_R = 64,
  SEXT_R = 96,
  /// means the register has not been written to ppcState yet.
  /// passing this to GetGPR signals that the guest register will be written there without fail
  FLAG_GUEST_UNSAVED = 128,
  /// this is used to avoid overwriting registers that are used by the current intruction
  FLAG_IN_USE = 256,
};
struct RegisterCache
{
  /// state of host registers at this point
  std::array<u16, 32> reg_state;
  u8 saved_regs = 0;

  RegisterCache()
  {
    // register 0 is special in addi and load/store, so better not use it for now
    reg_state[0] = RESERVED;
    // register 1 is always the stack pointer
    reg_state[1] = RESERVED;
    // register 2 is supposed to be the TOC pointer, but it's just a volatile register AFAICS
    reg_state[2] = RESERVED;
    // register 3 is the first argument, starts out as the JIT pointer, which we don't use
    reg_state[3] = FREE;
    // register 4 is the second argument, starts out as the offset value from the block DB (not used
    // currently)
    reg_state[4] = FREE;
    // register 5 is the third argument, starts out as the ppcState pointer
    reg_state[5] = PPCSTATE_PTR;
    // register 6 is the fourth argument, starts out as the ToC pointer
    reg_state[6] = TOC_PTR;
    // remaining volatile registers
    for (u8 i = 7; i < 13; ++i)
    {
      reg_state[i] = FREE;
    }
    // register 13 is the thread pointer per ABI
    reg_state[13] = RESERVED;
    // registers 14–31 are non-volatile, so start out unsaved
    for (u8 i = 14; i < 32; ++i)
    {
      reg_state[i] = HOST_UNSAVED;
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
    }
  }
  void FreeGPR(GPR host_gpr) { reg_state[host_gpr] = FREE; }
  GPR GetArgumentRegister(u8 index);
  GPR GetReturnRegister(u8 index);
  void BindGPR(GPR host_gpr, u16 specifier);
  void InvalidateAllRegisters();

  void EstablishStackFrame(PPC64BaselineCompiler* comp, u8 save_regs);
  void FlushHostRegister(PPCEmitter* emit, GPR gpr);
  void FlushAllRegisters(PPCEmitter* emit);
  void ReduceGuestRegisters(PPCEmitter* emit, u32 gprs_to_flush, u32 gprs_to_invalidate);

  GPR GetGPR(PPCEmitter* emit, u16 specifier);
  GPR GetScratch(PPCEmitter* emit, u16 specifier = SCRATCH);
  GPR GetPPCState();
  GPR GetToC();

  /// invalidates all non-LOCKED register references
  GPR PrepareCall(PPCEmitter* emit, u8 num_parameters);
  void BindCall(PPCEmitter* emit);
  /// invalidates all register references that are not LOCKED, TOC_PTR or PPCSTATE_PTR
  void PerformCall(PPCEmitter* emit, u8 num_return_gprs);
  /// invalidates all register references that are not LOCKED, TOC_PTR or PPCSTATE_PTR
  void PrepareReturn(PPCEmitter* emit, u8 num_return_values);

  void SetCR0(PPCEmitter* emit, GPR host_gpr);
};

}  // namespace PPC64RegCache
