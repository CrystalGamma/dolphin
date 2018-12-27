// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

class PPC64BaselineCompiler : public PPCEmitter
{
public:
  void Compile(const std::vector<u32> instructions);
};
