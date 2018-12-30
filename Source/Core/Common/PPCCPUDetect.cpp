// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/CPUDetect.h"

#include <asm/cputable.h>
#include <string>
#include <sys/auxv.h>
#include <unistd.h>

#include "Common/CommonTypes.h"
#include "Common/StringUtil.h"

CPUInfo cpu_info;

CPUInfo::CPUInfo()
{
  Detect();
}

void CPUInfo::Detect()
{
  vendor = CPUVendor::IBM;

  num_cores = static_cast<int>(sysconf(_SC_NPROCESSORS_CONF));
  long dcache_bs = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
  if (dcache_bs > 0)
  {
    dcache_blocksize = static_cast<size_t>(dcache_bs);
  }
  long icache_bs = sysconf(_SC_LEVEL1_ICACHE_LINESIZE);
  if (icache_bs > 0)
  {
    icache_blocksize = static_cast<size_t>(icache_bs);
  }

  long hwcap = getauxval(AT_HWCAP);
  if (hwcap & PPC_FEATURE_ICACHE_SNOOP)
  {
    icache_snoop = true;
  }
  if (hwcap & PPC_FEATURE_64)
  {
    OS64bit = true;
    CPU64bit = true;
    Mode64bit = true;
  }
}

// Turn the CPU info into a string we can show
std::string CPUInfo::Summarize()
{
  std::string str =
      StringFromFormat("PPC%d CPU, %d cores, I$ block size %zu, D$ block size %zu",
                       Mode64bit ? 64 : 32, num_cores, icache_blocksize, dcache_blocksize);
  if (icache_snoop)
  {
    str += ", I$ snoop supported";
  }
  return str;
}
