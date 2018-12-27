// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "Common/CodeBlock.h"

namespace PPCGen
{
class NativeEndianEmitter
{
public:
  NativeEndianEmitter();
  void Emit(const std::vector<u32>& instructions);
  /// flushes the specified range from dcache and issues invalidations for icache
  /// this needs to be called from the writing thread
  /// the executing thread will still need to issue 'isync' before jumping to this code
  void FlushCode(char* start, size_t length);

protected:
  void SetCodePtr(u8* ptr) { code = reinterpret_cast<u32*>(ptr); }
  u8* GetCodePtr() { return reinterpret_cast<u8*>(code); }

private:
  u32* code;
};

class PPCCodeSpace : public Common::CodeBlock<NativeEndianEmitter>
{
private:
  virtual void PoisonMemory() {}
};
}
