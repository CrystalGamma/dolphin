// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/PowerPC/JitTiered/JitTiered.h"

class JitTieredNoop final : public JitTieredCommon
{
public:
  virtual const char* GetName() const { return "Tiered Noop"; }
  virtual void BaselineCompile(u32 address, JitBlock&& block) {}
};
