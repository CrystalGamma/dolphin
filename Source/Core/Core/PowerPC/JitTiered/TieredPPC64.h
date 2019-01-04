// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/PPCCodeSpace.h"
#include "Core/PowerPC/JitTiered/JitTiered.h"
#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

class JitTieredPPC64 : public JitTieredCommon
{
public:
  JitTieredPPC64();
  virtual const char* GetName() const { return "TieredPPC64"; }

protected:
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry* entry, u32 address);
  virtual void BaselineCompile(u32 address, JitBlock&& block);
  virtual int GetHostCode(u32* address, const u8** code, u32* code_size) override;

private:
  void ReclaimCell(u32);
  /// maximum block size, in bytes (chosen to avoid wrapping (or needing to trampoline)
  /// conditional branches)
  static constexpr u32 MAX_BLOCK_SIZE = 1 << 15;
  /// size, in bytes, of a reclamation cell (must be at least MAX_BLOCK_SIZE + size of common
  /// routines)
  static constexpr u32 CODESPACE_CELL_SIZE = 1 << 16;
  /// distance, in cells, of common routines (must be exactly 2^26 bytes)
  static constexpr u32 ROUTINES_INTERVAL = 1 << 10;
  // number of cells to allocate (must be a multiple of ROUTINES_INTERVAL)
  static constexpr u32 CODESPACE_CELLS = ROUTINES_INTERVAL;

  u32 offset_in_cell = 0;
  u32 current_cell = 0;
  PPC64BaselineCompiler::CommonRoutineOffsets routine_offsets{};
  PPCCodeSpace codespace;
  PPC64BaselineCompiler::TableOfContents toc;
};
