// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/PPCCodeSpace.h"
#include "Core/PowerPC/JitTiered/JitTiered.h"

class JitTieredPPC64 : public JitTieredCommon
{
public:
  struct TableOfContents
  {
    void (*check_exceptions)();
    void (*check_external_exceptions)();
    u32 (*do_instruction)(UGeckoInstruction);
    std::array<InterpreterFunc, 4160> fallback_table;
  };
  JitTieredPPC64();
  virtual const char* GetName() const { return "TieredPPC64"; }

protected:
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry* entry, u32 address);
  virtual void BaselineCompile(u32 address, JitBlock&& block);

private:
  void ReclaimCell(u32);
  static constexpr u32 CODESPACE_CELL_SIZE = 1 << 13;
  static constexpr u32 CODESPACE_CELLS = 8;

  u32 offset_in_cell = 0;
  u32 current_cell = 0;
  PPCCodeSpace codespace;
  TableOfContents toc;
};
