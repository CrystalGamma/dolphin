// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitCommon/JitBase.h"

#ifdef _WIN32
#include <windows.h>
#else
#include "Common/PerformanceCounter.h"
#endif

#include "Common/CommonTypes.h"
#include "Core/Core.h"
#include "Core/ConfigManager.h"
#include "Core/HW/CPU.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/PowerPC/MMU.h"

JitBase* g_jit;

const u8* JitCommonBase::Dispatch(JitCommonBase& jit)
{
  return jit.GetBlockCache()->Dispatch();
}

void JitTrampoline(JitCommonBase& jit, u32 em_address)
{
  jit.Jit(em_address);
}

JitCommonBase::JitCommonBase() : m_code_buffer(code_buffer_size)
{
}

JitCommonBase::~JitCommonBase() = default;

bool JitCommonBase::CanMergeNextInstructions(int count) const
{
  if (CPU::IsStepping() || js.instructionsLeft < count)
    return false;
  // Be careful: a breakpoint kills flags in between instructions
  for (int i = 1; i <= count; i++)
  {
    if (SConfig::GetInstance().bEnableDebugging &&
        PowerPC::breakpoints.IsAddressBreakPoint(js.op[i].address))
      return false;
    if (js.op[i].isBranchTarget)
      return false;
  }
  return true;
}

void JitCommonBase::UpdateMemoryOptions()
{
  bool any_watchpoints = PowerPC::memchecks.HasAny();
  jo.fastmem = SConfig::GetInstance().bFastmem && (MSR.DR || !any_watchpoints);
  jo.memcheck = SConfig::GetInstance().bMMU || any_watchpoints;
}

void JitCommonBase::GetProfileResults(Profiler::ProfileStats* prof_stats) {
  Core::State old_state = Core::GetState();
  if (old_state == Core::State::Running)
    Core::SetState(Core::State::Paused);

  QueryPerformanceFrequency((LARGE_INTEGER*)&prof_stats->countsPerSec);
  GetBlockCache()->RunOnBlocks([&prof_stats](const JitBlock& block) {
    const auto& data = block.profile_data;
    u64 cost = data.downcountCounter;
    u64 timecost = data.ticCounter;
    // Todo: tweak.
    if (data.runCount >= 1)
      prof_stats->block_stats.emplace_back(block.effectiveAddress, cost, timecost, data.runCount,
                                           block.codeSize);
    prof_stats->cost_sum += cost;
    prof_stats->timecost_sum += timecost;
  });

  sort(prof_stats->block_stats.begin(), prof_stats->block_stats.end());
  if (old_state == Core::State::Running)
    Core::SetState(Core::State::Running);
}

using JitInterface::ExceptionType;
void JitCommonBase::CompileExceptionCheck(ExceptionType type)
{
  std::unordered_set<u32>* exception_addresses = nullptr;

  switch (type)
  {
  case ExceptionType::FIFOWrite:
    exception_addresses = &js.fifoWriteAddresses;
    break;
  case ExceptionType::PairedQuantize:
    exception_addresses = &js.pairedQuantizeAddresses;
    break;
  case ExceptionType::SpeculativeConstants:
    exception_addresses = &js.noSpeculativeConstantsAddresses;
    break;
  }

  if (PC != 0 && (exception_addresses->find(PC)) == (exception_addresses->end()))
  {
    if (type == ExceptionType::FIFOWrite)
    {
      // Check in case the code has been replaced since: do we need to do this?
      const OpType optype = PPCTables::GetOpInfo(PowerPC::HostRead_U32(PC))->type;
      if (optype != OpType::Store && optype != OpType::StoreFP && optype != OpType::StorePS)
        return;
    }
    exception_addresses->insert(PC);

    // Invalidate the JIT block so that it gets recompiled with the external exception check
    // included.
    GetBlockCache()->InvalidateICache(PC, 4, true);
  }
}
