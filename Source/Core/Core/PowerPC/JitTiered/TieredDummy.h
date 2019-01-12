// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/PowerPC/JitTiered/JitTiered.h"

class JitTieredDummy final : public JitTieredCommon
{
public:
  virtual void Init() override;
  virtual const char* GetName() const { return "Tiered Dummy"; }
  virtual void BaselineCompile(std::vector<u32> suggestions);

  static u32 Executor(JitTieredGeneric* self, u32 offset, PowerPC::PowerPCState* ppcState, void*);

private:
  void ProvideSpace(u32);
  static constexpr u32 CODESPACE_CELL_SIZE = 1 << 12;
  static constexpr u32 CODESPACE_CELLS = 4;
  std::array<DecodedInstruction, CODESPACE_CELL_SIZE * CODESPACE_CELLS> baseline_codespace;
};
