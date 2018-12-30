// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Common/PPCEmitter.h"
#include "Core/PowerPC/Gekko.h"

class PPC64BaselineCompiler : public PPCEmitter
{
public:
  void Compile(u32 address, const std::vector<UGeckoInstruction>& instructions);
};
