// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "Common/PPCEmitter.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/JitTiered/JitTiered.h"
#include "Core/PowerPC/JitTiered/PPC64RegCache.h"
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
    void* physical_base;
    void* logical_base;
    JitTieredGeneric::Executor interpreter_executor;
    JitTieredGeneric::DispatchCacheEntry* dispatch_cache;
    std::array<void (*)(UGeckoInstruction), 64 + 4 * 1024 + 32> fallback_table;
  };
  static_assert(offsetof(TableOfContents, logical_base) -
                        offsetof(TableOfContents, physical_base) ==
                    8,
                "JIT code requires the pointer to logical memmap 8 bytes after the pointer to "
                "physical memmap");
  struct CommonRoutineOffsets
  {
    u32 prologue;
    u32 dispatch_indirect;
    u32 dispatch_direct;
    u32 epilogue;
    u32 end;
  };
  PPC64BaselineCompiler() {}
  PPC64BaselineCompiler(CommonRoutineOffsets offs) : offsets{offs} {}
  void Compile(u32 addr, const std::vector<UGeckoInstruction>& instructions,
               const std::vector<JitTieredGeneric::Bail>& bails);
  void EmitCommonRoutines();
  void RelocateAll(u32 offset);

  CommonRoutineOffsets offsets{};
  std::vector<size_t> relocations;
  std::vector<std::pair<size_t, size_t>> fault_handlers;

private:
  enum : u32
  {
    JUMP = 0,
    LINK = 1,
    SKIP = 2,
    JUMPSPR = 4,
  };

  struct Exit
  {
    PPC64RegCache::RegisterCache reg_cache;
    u32 address;
    s32 downcount;
    u32 flags;
    u32 link_address;
  };

  struct FastmemHandler
  {
    FixupBranch location;
    PPC64RegCache::RegisterCache reg_cache;
    u32 address;
    s32 downcount;
  };

  struct ExceptionExit
  {
    FixupBranch branch;
    PPC64RegCache::RegisterCache reg_cache;
    u32 address;
    s32 downcount;
    u32 raise;
  };

  struct FallbackExit
  {
    PPC64RegCache::RegisterCache reg_cache;
    GPR pc;
    FixupBranch store_pc;
    FixupBranch leave_pc;
  };

  void RestoreRegisters(u32 saved_regs);
  void RestoreRegistersReturn(u32 saved_regs);

  void WriteExit(const Exit& jump);
  void WriteFastmemHandler(const FastmemHandler& handler);
  void WriteExceptionExit(const ExceptionExit& eexit);

  void FallbackToInterpreter(UGeckoInstruction inst, GekkoOPInfo& opinfo);
  void BCX(UGeckoInstruction inst, GekkoOPInfo& opinfo);
  void LoadStore(UGeckoInstruction inst, GekkoOPInfo& opinfo, const std::vector<u32>& bails);

  static constexpr s16 OFF_PC = s16(offsetof(PowerPC::PowerPCState, pc));
  static constexpr s16 OFF_NPC = s16(offsetof(PowerPC::PowerPCState, npc));
  static constexpr s16 OFF_DOWNCOUNT = s16(offsetof(PowerPC::PowerPCState, downcount));
  static constexpr s16 OFF_EXCEPTIONS = s16(offsetof(PowerPC::PowerPCState, Exceptions));

  u32 address = 0;
  s32 downcount = 0;
  PPC64RegCache::RegisterCache reg_cache;

  std::vector<std::pair<FixupBranch, Exit>> exits;
  std::vector<FallbackExit> fallback_exits;
  std::vector<FastmemHandler> fastmem_handlers;
  std::vector<ExceptionExit> exception_exits;
};
