// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/JitTiered.h"

#include <cinttypes>
#include <thread>
#include <utility>

#include "Common/Logging/Log.h"
#include "Core/CoreTiming.h"
#include "Core/HW/CPU.h"
#include "Core/PowerPC/PowerPC.h"

void JitTieredCommon::CPUDoReport(bool wait, bool hint)
{
  if (cpu_thread_lock.owns_lock())
  {
    cpu_thread_lock.unlock();
  }
  std::unique_lock lock = std::unique_lock(report_mutex, std::try_to_lock);
  if (wait)
  {
    if (!lock.owns_lock())
    {
      lock.lock();
    }
    if (report_sent)
    {
      report_cv.wait(lock, [&] { return !report_sent; });
    }
    // at this point, 'lock' owns a lock on report_mutex and report_sent == false
  }
  if (lock.owns_lock())
  {
    if (!report_sent)
    {
      // compaction needs access to dispatch cache
      cpu_thread_lock = std::unique_lock(disp_cache_mutex);
      CompactInterpreterBlocks(&baseline_report, false);
      baseline_report.invalidation_bloom = next_report.invalidation_bloom;
      baseline_report.invalidations.swap(next_report.invalidations);
      baseline_report.bails.swap(next_report.bails);
      old_bloom = next_report.invalidation_bloom;
      current_toc = next_toc;
      next_report.invalidation_bloom = BloomNone();
      report_sent = true;
      lock.unlock();
      // if the Baseline JIT thread is waiting for us, wake it up
      report_cv.notify_one();
    }
    else
    {
      lock.unlock();
    }
  }
  if (on_thread_baseline && (wait || hint))
  {
    if (cpu_thread_lock.owns_lock())
    {
      cpu_thread_lock.unlock();
    }
    BaselineIteration();
  }
}

void JitTieredCommon::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  std::thread compiler_thread;
  if (!on_thread_baseline)
  {
    compiler_thread = std::thread([this] {
      while (BaselineIteration())
      {
      }
    });
  }
  while (*state == CPU::State::Running)
  {
    // advancing the time might block (e. g. on the GPU)
    // or keep us busy for a while (copying data around),
    // so try to report before doing so
    CPUDoReport(false, next_report.instructions.size() >= (1 << 16));

    CoreTiming::Advance();
    PowerPC::CheckExternalExceptions();
    if (!cpu_thread_lock.owns_lock())
    {
      cpu_thread_lock = std::unique_lock(disp_cache_mutex);
    }

    do
    {
      const u32 start_addr = PC;
      DispatchCacheEntry* cache_entry = FindBlock(start_addr);
      const u32 usecount = cache_entry->usecount + 1;
      cache_entry->usecount = usecount;
      const u32 flags =
          cache_entry->executor(this, cache_entry->offset, &PowerPC::ppcState, current_toc);
      if (flags & BLOCK_OVERRUN)
      {
        if (flags & REPORT_BAIL)
        {
          next_report.bails.push_back({PC, flags});
        }
        HandleOverrun(cache_entry);
      }
      if (flags & REPORT_BAIL)
      {
        next_report.bails.push_back({PC, flags});
      }
      if ((cache_entry->executor == interpreter_executor && usecount >= REPORT_THRESHOLD) ||
          flags & (REPORT_IMMEDIATELY | REPORT_BAIL))
      {
        CPUDoReport(flags & REPORT_IMMEDIATELY, flags & REPORT_BAIL);
        if (!cpu_thread_lock.owns_lock())
        {
          cpu_thread_lock = std::unique_lock(disp_cache_mutex);
        }
      }
      PowerPC::CheckExceptions();
      PowerPC::CheckBreakPoints();
    } while (PowerPC::ppcState.downcount > 0 && *state == CPU::State::Running);
  }
  if (cpu_thread_lock.owns_lock())
  {
    cpu_thread_lock.unlock();
  }

  if (compiler_thread.joinable())
  {
    NOTICE_LOG(DYNA_REC, "Shutting down JIT compiler");

    // shut down compiler thread
    cpu_thread_lock = std::unique_lock(report_mutex);
    quit = true;
    report_cv.notify_one();
    report_cv.wait(cpu_thread_lock, [&] { return quit == false; });
    cpu_thread_lock.unlock();
    compiler_thread.join();
  }
}

void JitTieredCommon::HandleOverrun(DispatchCacheEntry* cache_entry)
{
  const u32 start_addr = cache_entry->address;

  u32 offset = next_report.instructions.size();
  u32 len = 0;
  if (cache_entry->executor != interpreter_executor)
  {
    INFO_LOG(DYNA_REC, "JIT block overrun @ %08x", start_addr);
    next_report.invalidation_bloom |= BloomCacheline(start_addr);
    {
      std::lock_guard lock(block_db_mutex);
      // we're on the CPU thread and know the block hasn't been invalidated, so no need to deal with
      // bloom filters
      auto find_result = jit_block_db.find(start_addr);
      // Baseline must not erase the entry while it is in dispatch cache
      ASSERT(find_result != jit_block_db.end());
      const std::vector<DecodedInstruction>& instructions = find_result->second.instructions;
      len = instructions.size();
      for (const DecodedInstruction& inst : instructions)
      {
        next_report.instructions.push_back(inst);
      }
      next_report.instructions.push_back({});
    }

    // address is already correct
    cache_entry->offset = offset;
    cache_entry->length = len;
    cache_entry->executor = interpreter_executor;
    // bloom is already correct
    // leave usecount untouched

    // JIT bails may leave us in the middle of a block
    if (PC < start_addr + len * 4)
    {
      ASSERT(PC >= start_addr);
      if (!InterpretBlock(&next_report.instructions[offset + (PC - start_addr) / 4]))
      {
        return;
      }
    }
    ASSERT(PC == start_addr + len * 4);
  }
  JitTieredGeneric::HandleOverrun(cache_entry);
}

JitTieredGeneric::DispatchCacheEntry* JitTieredCommon::LookupBlock(DispatchCacheEntry* cache_entry,
                                                                   u32 address)
{
  std::unique_lock blockdb_lock(block_db_mutex);

  auto find_result = jit_block_db.find(address);
  Bloom bloom = next_report.invalidation_bloom | old_bloom;
  if (find_result == jit_block_db.end() || find_result->second.executor == nullptr ||
      (find_result->second.bloom & bloom))
  {
    blockdb_lock.unlock();

    return JitTieredGeneric::LookupBlock(cache_entry, address);
  }
  else
  {
    JitBlock& block = find_result->second;
    DispatchCacheEntry entry;
    entry.address = address;
    entry.offset = block.offset;
    entry.bloom = block.bloom;
    entry.executor = block.executor;
    entry.length = block.instructions.size();
    entry.usecount = 0;

    for (auto& handler : block.fault_handlers)
    {
      INFO_LOG(DYNA_REC, "installing fault handler: 0x%" PRIxPTR " ↦ 0x%" PRIxPTR, handler.first,
               handler.second);
      fault_handlers.insert(handler);
    }

    blockdb_lock.unlock();
    INFO_LOG(DYNA_REC, "found JIT block @ %08x (offset %x)", address, entry.offset);

    *cache_entry = entry;
    return cache_entry;
  }
}

bool JitTieredCommon::HandleFault(uintptr_t access_address, SContext* ctx)
{
  // FIXME: check if we are on the CPU thread
  if (!cpu_thread_lock.owns_lock())
  {
    // no JIT code running
    return false;
  }
  // we have a lock on disp_cache_mutex, so we can look up the fault handlers
  auto find_result = fault_handlers.find(ctx->CTX_NPC);
  if (find_result == fault_handlers.end())
  {
    ERROR_LOG(DYNA_REC,
              "Segfault without handler accessing 0x%" PRIx64 " @ 0x%" PRIx64 ", crashing now",
              uintptr_t(access_address), uintptr_t(ctx->CTX_NPC));
    return false;
  }
  ctx->CTX_NPC = find_result->second;
  return true;
}

void JitTieredCommon::UpdateBlockDB(Bloom bloom, std::vector<Invalidation>* invalidations,
                                    std::vector<Bail>* bails,
                                    std::map<u32, ReportedBlock>* reported_blocks)
{
  std::sort(invalidations->begin(), invalidations->end(),
            [](const Invalidation& a, const Invalidation& b) {
              return std::tie(a.first, a.last) < std::tie(b.first, b.last);
            });
  std::sort(bails->begin(), bails->end(), [](const Bail& a, const Bail& b) {
    return std::tie(a.guest_address, a.status) < std::tie(b.guest_address, b.status);
  });
  std::unique_lock guard(block_db_mutex);
  size_t num_inv = invalidations->size();
  size_t inv_start = 0;
  size_t num_bails = bails->size();
  size_t bail_start = 0;
  auto iter = jit_block_db.begin();
  // this loop should run in O(num_inv + num_bails + #blocks * log(#reported_blocks) * (size of
  // biggest intersecting cluster of invalidations)^2) time if this turns out to be too slow, we may
  // need to find a container more suitable to finding intersecting ranges
  while (iter != jit_block_db.end())
  {
    const u32 addr = iter->first;
    JitBlock& block = iter->second;
    if (block.executor == nullptr)
    {
      // clean up blocks that were invalidated last cycle but not recompiled
      iter = jit_block_db.erase(iter);
      continue;
    }
    const u32 jit_len = block.instructions.size();
    const bool bloom_hit = block.bloom & baseline_report.invalidation_bloom;
    while (inv_start < num_inv && invalidations->at(inv_start).last < addr)
    {
      // this and all previous blocks only affect lower addresses
      inv_start += 1;
    }
    while (bail_start < num_bails && bails->at(bail_start).guest_address < addr)
    {
      // this and all previous blocks only affect lower addresses
      bail_start += 1;
    }
    auto reported_block = reported_blocks->find(addr);
    u32 reported_len = 0;
    if (reported_block != reported_blocks->end())
    {
      reported_len = reported_block->second.len;
      if (reported_len == 0)
      {
        // blocks with zero length are considered optimized blocks, so make sure we don't try to
        // compile a zero-length block
        reported_block->second.usecount += block.runcount;
        reported_blocks->erase(reported_block);
      }
    }
    bool invalidated = false, bailed = false;
    if (jit_len != 0)
    {
      for (size_t i = bail_start; i < num_bails; ++i)
      {
        const Bail& bail = bails->at(i);
        if (bail.guest_address >= addr + jit_len * 4)
        {
          // this and all following are at higher addresses
          break;
        }
        bailed = true;
        block.bails.push_back(bail);
      }
      if (reported_len > jit_len)
      {
        // Baseline block overrun
        invalidated = true;
      }
      else if (bloom_hit)
      {
        for (size_t i = inv_start; i < num_inv; i += 1)
        {
          const Invalidation& inv = invalidations->at(i);
          if (inv.first >= addr + jit_len * 4)
          {
            // this and all following invalidations affect only higher addresses
            break;
          }
          if (inv.last >= addr)
          {
            invalidated = true;
            break;
          }
        }
      }
    }
    else if (bloom_hit)
    {
      // figure out how to check optimized blocks for invalidation and bails when implementing
      // optimized JIT
      invalidated = true;
      break;
    }
    if (invalidated || bailed)
    {
      if (reported_len != 0)
      {
        INFO_LOG(DYNA_REC, "recompile Baseline block @ %08x", addr);
        reported_block->second.usecount += block.runcount;
        std::sort(block.bails.begin(), block.bails.end(), [](const Bail& a, const Bail& b) {
          return std::tie(a.guest_address, a.status) < std::tie(b.guest_address, b.status);
        });
        reported_block->second.bails.swap(block.bails);
      }
      else
      {
        INFO_LOG(DYNA_REC, "invalidate JIT block @ %08x (used %" PRIu64 " times)", addr,
                 block.runcount);
        block_counters.emplace(addr, block.runcount);
      }
      if (invalidated)
      {
        block.executor = nullptr;
      }
    }
    else
    {
      block.runcount += reported_block->second.usecount;
    }
    ++iter;
  }
  invalidations->clear();
  bails->clear();
}

bool JitTieredCommon::BaselineIteration()
{
  std::unique_lock lock(report_mutex);
  report_cv.wait(lock, [&] { return report_sent || quit; });

  report_sent = false;
  if (quit)
  {
    quit = false;
    lock.unlock();
    report_cv.notify_one();
    return false;
  }

  INFO_LOG(DYNA_REC, "reading report");

  std::map<u32, ReportedBlock> reported_blocks;
  for (const DispatchCacheEntry& block : baseline_report.blocks)
  {
    const ReportedBlock rb = {
        block.offset, block.executor == interpreter_executor ? block.length : 0, block.usecount};
    reported_blocks.emplace(block.address, rb);
  }
  baseline_report.blocks.clear();

  UpdateBlockDB(baseline_report.invalidation_bloom, &baseline_report.invalidations,
                &baseline_report.bails, &reported_blocks);
  baseline_report.invalidation_bloom = BloomNone();

  for (auto pair : reported_blocks)
  {
    const u32 address = pair.first;
    const ReportedBlock& block = pair.second;
    u64 runcount = block.usecount;
    auto counter = block_counters.find(address);
    if (counter != block_counters.end())
    {
      runcount += counter->second;
      if (runcount < BASELINE_THRESHOLD)
      {
        counter->second = runcount;
        continue;
      }
      block_counters.erase(counter);
    }
    else
    {
      if (runcount < BASELINE_THRESHOLD)
      {
        block_counters.insert(std::make_pair(address, runcount));
        continue;
      }
    }
    if (block.len == 0)
    {
      // entries with length 0 in the block are considered 'optimized' blocks
      // and trying to compile an empty block doesn't make sense anyway
      continue;
    }
    std::vector<DecodedInstruction> insts;
    for (u32 i = 0; i < block.len; i += 1)
    {
      insts.push_back(baseline_report.instructions[block.start + i]);
    }
    JitBlock jb = {BloomRange(address, address + 4 * block.len - 1),
                   nullptr,
                   runcount,
                   current_offset,
                   std::move(insts),
                   std::move(block.bails),
                   {},
                   0};
    BaselineCompile(address, std::move(jb));
  }

  // the mutex on the report drops here, so next time the CPU thread reports,
  // it will drop the interpreter blocks whose instructions we just received,
  // to be replaced by our JIT code once that address is jumped to next.
  lock.unlock();
  // in the (hopefully unlikely) case that the CPU thread is waiting on us, notify it
  report_cv.notify_one();

  // TODO: analyse instrumentation data, build CFGs/dominator trees, optimize
  return true;
}
