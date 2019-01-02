// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "Common/PPCEmitter.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/PowerPC.h"

class PPC64BaselineCompiler final : public PPCEmitter
{
public:
  struct TableOfContents
  {
    void (*idle)();
    u32 (*load_word)(u32 addr);
    void (*store_word)(u32 val, u32 addr);
    u8 (*load_byte)(u32 addr);
    void (*store_byte)(u8 val, u32 addr);
    u16 (*load_hword)(u32 addr);
    s16 (*load_hword_sext)(u32 addr);
    void (*store_hword)(u16 val, u32 addr);
    std::array<void (*)(UGeckoInstruction), 64 + 4 * 1024 + 32> fallback_table;
  };
  struct CommonRoutineOffsets
  {
    u32 save_gprs;
    u32 restore_gpr_return;
    u32 restore_gpr;
    u32 end;
  };
  PPC64BaselineCompiler() {}
  PPC64BaselineCompiler(CommonRoutineOffsets offs) : offsets{offs} {}
  void Compile(u32 addr, const std::vector<UGeckoInstruction>& instructions);
  void EmitCommonRoutines();
  void RelocateAll(u32 offset);

  CommonRoutineOffsets offsets{};
  std::vector<size_t> relocations;

private:
  enum : u32
  {
    JUMP = 0,
    LINK = 1,
    SKIP = 2,
    JUMPSPR = 4,
    EXCEPTION = 8,
    RAISE_FPU_EXC = 16
  };

  struct Exit
  {
    u32 address;
    s32 downcount;
    u32 flags;
    u32 link_address;
  };

  struct FallbackExit
  {
    FixupBranch store_pc;
    FixupBranch leave_pc;
    s32 downcount;
  };

  static constexpr s16 GPROffset(u32 i)
  {
    return s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * i);
  }

  static constexpr s16 SPROffset(u32 i)
  {
    return s16(offsetof(PowerPC::PowerPCState, spr) + 4 * i);
  }

  void RestoreRegisters(u32 saved_regs);
  void RestoreRegistersReturn(u32 saved_regs);

  void WriteExit(const Exit& jump);

  void FallbackToInterpreter(UGeckoInstruction inst, GekkoOPInfo& opinfo);
  void BCX(UGeckoInstruction inst, GekkoOPInfo& opinfo);
  void DFormLoadStore(UGeckoInstruction inst, GekkoOPInfo& opinfo);

  static constexpr GPR PPCSTATE = R30;
  static constexpr GPR TOC = R29;
  static constexpr GPR SCRATCH1 = R7;
  static constexpr GPR SCRATCH2 = R8;
  static constexpr GPR SAVED1 = R31;
  static constexpr GPR ARG1 = R3;
  static constexpr GPR ARG2 = R4;
  static constexpr s16 OFF_PC = s16(offsetof(PowerPC::PowerPCState, pc));
  static constexpr s16 OFF_DOWNCOUNT = s16(offsetof(PowerPC::PowerPCState, downcount));
  static constexpr s16 OFF_EXCEPTIONS = s16(offsetof(PowerPC::PowerPCState, Exceptions));

  u32 address = 0;
  s32 downcount = 0;

  std::vector<std::pair<FixupBranch, Exit>> exits;
  std::vector<FallbackExit> fallback_exits;
};
