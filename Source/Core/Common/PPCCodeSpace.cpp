// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/PPCCodeSpace.h"

#include <cstring>

#include "Common/CPUDetect.h"

void NativeEndianEmitter::Emit(const std::vector<u32>& instructions)
{
  std::memcpy(code, instructions.data(), instructions.size() * 4);

  FlushCode(reinterpret_cast<char*>(code), instructions.size() * 4);
  code += instructions.size();
}

void NativeEndianEmitter::FlushCode(char* start, size_t length)
{
  // on POWER9, data cache is coherent
  if (!cpu_info.icache_snoop)
  {
    char* ptr = start;
    size_t num_blocks = (length + cpu_info.dcache_blocksize - 1) / cpu_info.dcache_blocksize;
    for (size_t i = 0; i < num_blocks; i++)
    {
      asm volatile("dcbst 0, %0" : : "r"(ptr));
      ptr += cpu_info.dcache_blocksize;
    }
  }
  asm volatile("sync" : :);
  if (!cpu_info.icache_snoop)
  {
    char* ptr = start;
    size_t num_blocks = (length + cpu_info.icache_blocksize - 1) / cpu_info.icache_blocksize;
    for (size_t i = 0; i < num_blocks; i++)
    {
      asm volatile("icbi 0, %0" : : "r"(ptr));
      ptr += cpu_info.icache_blocksize;
    }
  }
  else
  {
    // on POWER9, any address will do, use the given pointer
    asm volatile("icbi 0, %0" : : "r"(start));
  }
}
