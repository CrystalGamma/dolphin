// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/PowerPC/JitTiered/JitTiered.h"

class JitTieredPPC64 : public JitTieredCommon
{
public:
  JitTieredPPC64();
  virtual const char* GetName() const { return "TieredPPC64"; }

protected:
  virtual u32 LookupBlock(u32 key, u32 address);
  virtual void BaselineCompile(u32 address, JitBlock&& block);

private:
  static constexpr CODESPACE_CELL_SIZE = 1 << 13;
  static constexpr CODESPACE_CELLS = 8;

  u32 offset_in_cell = 0;
  u32 cell = 0;
  PPCCodeBlock codespace;
};
