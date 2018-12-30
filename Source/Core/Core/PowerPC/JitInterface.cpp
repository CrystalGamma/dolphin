// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitInterface.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <string>
#include <unordered_set>

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/File.h"
#include "Common/MsgHandler.h"

#include "Core/PowerPC/CPUCoreBase.h"
#include "Core/PowerPC/CachedInterpreter/CachedInterpreter.h"
#include "Core/PowerPC/JitCommon/JitBase.h"
#include "Core/PowerPC/JitTiered/JitTiered.h"
#include "Core/PowerPC/JitTiered/TieredDummy.h"
#include "Core/PowerPC/JitTiered/TieredNoop.h"
#include "Core/PowerPC/MMU.h"
#include "Core/PowerPC/PPCSymbolDB.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/Profiler.h"

#if _M_X86
#include "Core/PowerPC/Jit64/Jit.h"
#endif

#if _M_ARM_64
#include "Core/PowerPC/JitArm64/Jit.h"
#endif

#if _M_PPC64
#include "Core/PowerPC/JitTiered/TieredPPC64.h"
#endif

namespace JitInterface
{
static JitBase* g_jit = nullptr;
void SetJit(JitBase* jit)
{
  g_jit = jit;
}
void DoState(PointerWrap& p)
{
  if (g_jit && p.GetMode() == PointerWrap::MODE_READ)
    g_jit->ClearCache();
}
CPUCoreBase* InitJitCore(PowerPC::CPUCore core)
{
  switch (core)
  {
#if _M_X86
  case PowerPC::CPUCore::JIT64:
    g_jit = new Jit64();
    break;
#endif
#if _M_ARM_64
  case PowerPC::CPUCore::JITARM64:
    g_jit = new JitArm64();
    break;
#endif
  case PowerPC::CPUCore::CachedInterpreter:
    g_jit = new CachedInterpreter();
    break;
  case PowerPC::CPUCore::TieredNoop:
    g_jit = new JitTieredNoop();
    break;
  case PowerPC::CPUCore::TieredGeneric:
    g_jit = new JitTieredGeneric();
    break;
  case PowerPC::CPUCore::TieredDummy:
    g_jit = new JitTieredDummy();
    break;
#if _M_PPC64
  case PowerPC::CPUCore::TieredPPC64:
    g_jit = new JitTieredPPC64();
    break;
#endif

  default:
    PanicAlertT("The selected CPU emulation core (%d) is not available. "
                "Please select a different CPU emulation core in the settings.",
                static_cast<int>(core));
    g_jit = nullptr;
    return nullptr;
  }
  g_jit->Init();
  return g_jit;
}

CPUCoreBase* GetCore()
{
  return g_jit;
}

void EnableProfiling(bool enabled)
{
  if (g_jit)
  {
    g_jit->EnableProfiling(enabled);
  }
}

static void GetProfileResults(Profiler::ProfileStats* prof_stats)
{
  if (g_jit)
  {
    g_jit->GetProfileResults(prof_stats);
  }
}

void WriteProfileResults(const std::string& filename)
{
  Profiler::ProfileStats prof_stats;
  GetProfileResults(&prof_stats);

  File::IOFile f(filename, "w");
  if (!f)
  {
    PanicAlert("Failed to open %s", filename.c_str());
    return;
  }
  fprintf(f.GetHandle(), "origAddr\tblkName\trunCount\tcost\ttimeCost\tpercent\ttimePercent\tOvAlli"
                         "nBlkTime(ms)\tblkCodeSize\n");
  for (auto& stat : prof_stats.block_stats)
  {
    std::string name = g_symbolDB.GetDescription(stat.addr);
    double percent = 100.0 * (double)stat.cost / (double)prof_stats.cost_sum;
    double timePercent = 100.0 * (double)stat.tick_counter / (double)prof_stats.timecost_sum;
    fprintf(f.GetHandle(),
            "%08x\t%s\t%" PRIu64 "\t%" PRIu64 "\t%" PRIu64 "\t%.2f\t%.2f\t%.2f\t%i\n", stat.addr,
            name.c_str(), stat.run_count, stat.cost, stat.tick_counter, percent, timePercent,
            (double)stat.tick_counter * 1000.0 / (double)prof_stats.countsPerSec, stat.block_size);
  }
}

int GetHostCode(u32* address, const u8** code, u32* code_size)
{
  if (!g_jit)
  {
    *code_size = 0;
    return 1;
  }
  return g_jit->GetHostCode(address, code, code_size);
}

bool HandleFault(uintptr_t access_address, SContext* ctx)
{
  // Prevent nullptr dereference on a crash with no JIT present
  if (!g_jit)
  {
    return false;
  }

  return g_jit->HandleFault(access_address, ctx);
}

bool HandleStackFault()
{
  if (!g_jit)
  {
    return false;
  }

  return g_jit->HandleStackFault();
}

void ClearCache()
{
  if (g_jit)
    g_jit->ClearCache();
}
void ClearSafe()
{
  if (g_jit)
    g_jit->ClearSafe();
}

void InvalidateICache(u32 address, u32 size, bool forced)
{
  if (g_jit)
    g_jit->InvalidateICache(address, size, forced);
}

void CompileExceptionCheck(ExceptionType type)
{
  if (g_jit)
    g_jit->CompileExceptionCheck(type);
}

void Shutdown()
{
  if (g_jit)
  {
    g_jit->Shutdown();
    delete g_jit;
    g_jit = nullptr;
  }
}
}
