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

u32 JitTieredCommon::CheckBPAndInterpret(JitTieredGeneric* self, u32 offset,
                                         PowerPC::PowerPCState* ppcState, void* toc)
{
  // on baseline code checking for breakpoints can take ≥10% of the CPU thread time if it is done at
  // every block. instead we only check on interpreter blocks (which are slow anyway) and reject JIT
  // blocks that contain breakpoints
  if (PowerPC::breakpoints.IsAddressBreakPoint(PC))
  {
    JitTieredCommon* xself = static_cast<JitTieredCommon*>(self);
    if (!xself->breakpoint_handled)
    {
      // we haven't stopped at this breakpoint yet
      PowerPC::CheckBreakPoints();
      xself->breakpoint_handled = true;
      return 0;
    }
    else
    {
      // we already stopped for this one, stop again at the next breakpoint
      xself->breakpoint_handled = false;
    }
  }
  return InterpreterExecutor(self, offset, ppcState, toc);
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
  CoreTiming::Advance();
  while (*state == CPU::State::Running)
  {
    const u32 start_addr = PC;
    DispatchCacheEntry* cache_entry = FindBlock(start_addr);
    const u32 usecount = cache_entry->usecount + 1;
    cache_entry->usecount = usecount;
    u32 flags = cache_entry->executor(this, cache_entry->offset, &PowerPC::ppcState, current_toc);
    if (flags & REPORT_BAIL)
    {
      next_report.bails.push_back({PC, flags});
      flags = HandleBail(PC);
    }
    bool jump_out = flags & JUMP_OUT;
    if (flags & BLOCK_OVERRUN)
    {
      if (cache_entry->executor == interpreter_executor)
      {
        jump_out = HandleOverrun(cache_entry);
      }
      else
      {
        // this case is super-rare: to create this scenario, the block would need to be executed
        // often enough to be JITted, but exit with an exception every time, but once it is compiled
        // and discovered by the CPU thread, run without exception.
        WARN_LOG(DYNA_REC, "true JIT overrun @ %08x", PC);
        std::vector<DecodedInstruction> insts;
        u32 address;
        {
          std::lock_guard<std::mutex> guard(block_db_mutex);
          auto iter = jit_block_db.upper_bound(PC);
          ASSERT(iter != jit_block_db.begin());
          --iter;
          address = iter->first;
          ASSERT((PC - address) / 4 <= iter->second.instructions.size());
          insts = iter->second.instructions;
        }
        // FIXME: do this better
        u32 key = DispatchCacheKey(address);
        DispatchCacheEntry& overrun_entry = dispatch_cache[key];
        if (!overrun_entry.IsValid() || overrun_entry.address != address)
        {
          overrun_entry.address = address;
          overrun_entry.executor = interpreter_executor;
          ASSERT(!next_report.instructions.empty());
          overrun_entry.offset = next_report.instructions.size();
          overrun_entry.length = insts.size();
          overrun_entry.bloom = BloomRange(address, address + 4 * insts.size());
          overrun_entry.usecount = 0;
          for (auto inst : insts)
          {
            next_report.instructions.push_back(inst);
          }
          next_report.instructions.push_back({});
        }
        u32 res = InterpretBlock(&next_report.instructions[(PC - address) / 4]);
        if (res & BLOCK_OVERRUN)
        {
          jump_out = HandleOverrun(&overrun_entry);
        }
        else
        {
          jump_out = res & JUMP_OUT;
        }
      }
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
    if (jump_out)
    {
      if (PowerPC::ppcState.Exceptions)
      {
        PowerPC::CheckExceptions();
      }
      if (PowerPC::ppcState.downcount <= 0)
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
      }
    }
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

u32 JitTieredCommon::HandleBail(u32 start_addr)
{
  std::vector<DecodedInstruction> buf;
  buf.push_back({});
  {
    std::lock_guard lock(block_db_mutex);
    // look up the basic block we're in
    auto iter = jit_block_db.upper_bound(start_addr);
    // bails should leave us in the middle of a known block
    ASSERT(iter != jit_block_db.begin());
    --iter;
    const std::vector<DecodedInstruction>& instructions = iter->second.instructions;
    u32 start = (PC - iter->first) / 4;
    ASSERT(start < instructions.size());
    for (u32 i = start; i < instructions.size(); ++i)
    {
      buf.push_back(instructions[i]);
    }
  }
  buf.push_back({});
  return InterpretBlock(&buf[0]);
}

JitTieredGeneric::DispatchCacheEntry* JitTieredCommon::LookupBlock(DispatchCacheEntry* cache_entry,
                                                                   u32 address)
{
  if (PowerPC::breakpoints.IsAddressBreakPoint(address))
  {
    // reject blocks at breakpoints
    return JitTieredGeneric::LookupBlock(cache_entry, address);
  }

  std::unique_lock blockdb_lock(block_db_mutex);

  auto find_result = jit_block_db.find(address);
  if (find_result == jit_block_db.end())
  {
    blockdb_lock.unlock();

    return JitTieredGeneric::LookupBlock(cache_entry, address);
  }

  DispatchCacheEntry entry{};
  entry.address = address;
  entry.usecount = 0;

  Bloom bloom = next_report.invalidation_bloom | old_bloom;
  const CompiledBlock* jit_block = nullptr;
  for (const CompiledBlock& block : find_result->second.entry_points)
  {
    bool contains_breakpoint = false;
    for (u32 bb_addr : block.additional_bbs)
    {
      if (PowerPC::breakpoints.IsAddressBreakPoint(bb_addr))
      {
        WARN_LOG(DYNA_REC, "rejecting block @ %08x (breakpoint @ %08x)", address, bb_addr);
        contains_breakpoint = true;
        break;
      }
    }
    if (!contains_breakpoint && !(block.bloom & bloom))
    {
      jit_block = &block;
      break;
    }
    WARN_LOG(DYNA_REC, "rejecting block @ %08x", address);
  }
  if (jit_block != nullptr)
  {
    // found a JIT block that is not matched by our invalidation bloom filter nor contains
    // breakpoints
    entry.executor = jit_block->executor;
    entry.bloom = jit_block->bloom;
    entry.offset = jit_block->offset;
    entry.length = jit_block->guest_length;
    INFO_LOG(DYNA_REC, "found JIT block @ %08x (offset %x, length %u)", address, entry.offset,
             entry.length);

    for (auto& handler : jit_block->fault_handlers)
    {
      INFO_LOG(DYNA_REC, "installing fault handler: 0x%" PRIxPTR " ↦ 0x%" PRIxPTR, handler.first,
               handler.second);
      fault_handlers.insert(handler);
    }
  }
  else
  {
    // we can still use the instructions in the code model to build an interpreter block
    // (note that we won't miss breakpoints this way, because those would end the basic block)
    entry.executor = interpreter_executor;
    entry.length = find_result->second.instructions.size();
    entry.bloom = BloomRange(address, address + entry.length - 1);

    if (next_report.instructions.empty())
    {
      next_report.instructions.push_back({});
    }
    entry.offset = next_report.instructions.size();
    for (const DecodedInstruction& inst : find_result->second.instructions)
    {
      next_report.instructions.push_back(inst);
    }
    next_report.instructions.push_back({});
    INFO_LOG(DYNA_REC, "found block @ %08x (offset %x, length %u)", address, entry.offset,
             entry.length);
  }

  blockdb_lock.unlock();

  *cache_entry = entry;
  return cache_entry;
}

std::vector<JitTieredCommon::BaselineCompileJob>
JitTieredCommon::PrepareBaselineSuggestions(std::vector<u32> suggestions)
{
  std::sort(suggestions.begin(), suggestions.end());

  std::vector<BaselineCompileJob> jobs;

  // the Baseline thread can do unsynchronized reads on the code model
  const std::map<u32, BasicBlock>& code_model = jit_block_db;
  u32 pos = 0;
  while (pos < suggestions.size())
  {
    const u32 address = suggestions[pos];
    std::vector<DecodedInstruction> buf;
    std::vector<u32> additional_bbs;
    // pull in all the code until the next unconditional jump / context synchronizing instruction
    do
    {
      const u32 this_bb = address + 4 * buf.size();
      if (pos < suggestions.size() && suggestions[pos] == this_bb)
      {
        // skip this suggestion, since we're already building it into our block. if it turns out
        // this block is entered from a different block, it will come up again eventually
        ++pos;
      }
      auto iter = code_model.find(this_bb);
      if (iter == code_model.cend())
      {
        break;
      }
      if (!buf.empty())
      {
        additional_bbs.push_back(this_bb);
      }
      for (auto inst : iter->second.instructions)
      {
        buf.push_back(inst);
      }
      // assume we have to go to dispatcher if the BB didn't end in a conditional branch
    } while (buf.back().inst.OPCD != 16);
    jobs.push_back({address, std::move(buf), std::move(additional_bbs)});
  }
  return std::move(jobs);
}

void JitTieredCommon::AddJITBlock(u32 address, CompiledBlock block)
{
  std::lock_guard<std::mutex> lock(block_db_mutex);
  for (u32 addr : block.additional_bbs)
  {
    auto iter = jit_block_db.find(addr);
    ASSERT(iter != jit_block_db.end());
    iter->second.inlined_in.push_back(address);
    iter->second.runcount = 0;
  }
  auto iter = jit_block_db.find(address);
  ASSERT(iter != jit_block_db.end());
  iter->second.entry_points.push_back(std::move(block));
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
  std::vector<u32> baseline_suggestions;
  {
    std::lock_guard guard(block_db_mutex);
    for (auto inv : baseline_report.invalidations)
    {
      auto iter = jit_block_db.upper_bound(inv.last);
      while (iter != jit_block_db.begin())
      {
        --iter;
        if (iter->first + iter->second.instructions.size() <= inv.first)
        {
          break;
        }
        std::vector<u32> inlined_in{};
        iter->second.inlined_in.swap(inlined_in);
        iter = jit_block_db.erase(iter);
        for (u32 additional_bb : inlined_in)
        {
          auto iter2 = jit_block_db.find(additional_bb);
          if (iter2 != jit_block_db.end())
          {
            iter2->second.entry_points.clear();
          }
        }
      }
      auto bail = all_bails.lower_bound(Bail{inv.first, 0});
      while (bail != all_bails.end() && bail->guest_address <= inv.last)
      {
        bail = all_bails.erase(bail);
      }
    }
    baseline_report.invalidation_bloom = BloomNone();
    baseline_report.invalidations.clear();

    for (auto report_block : baseline_report.blocks)
    {
      if (report_block.executor != interpreter_executor || report_block.length == 0)
      {
        continue;
      }
      auto iter = jit_block_db.upper_bound(report_block.address);
      u32 max_length = MAX_BLOCK_LENGTH;
      if (iter == jit_block_db.end())
      {
        // no block after this ⇒ limit to end of address space
        const u32 available_space = (0xfffffffc - report_block.address) / 4 + 1;
        if (max_length > available_space)
        {
          max_length = available_space;
        }
      }
      else
      {
        // limit to beginning of next basic block
        const u32 available_space = (iter->first - report_block.address) / 4;
        if (max_length > available_space)
        {
          max_length = available_space;
        }
      }
      if (iter != jit_block_db.begin())
      {
        auto insert_pos = iter;
        --iter;
        // since we now the basic block after this is the first at higher address than the new
        // block, this never underflows
        u32 split_pos = (report_block.address - iter->first) / 4;
        BasicBlock& old_block = iter->second;
        if (split_pos >= old_block.instructions.size())
        {
          // completely new block
          iter = jit_block_db.emplace_hint(insert_pos, report_block.address,
                                           BasicBlock{{}, 0, {}, {}});
        }
        else if (split_pos > 0)
        {
          // split block: save old instructions in case the old one exited early (e. g. exceptions)
          std::vector<DecodedInstruction> old_instructions;
          for (u32 i = split_pos; i < old_block.instructions.size(); ++i)
          {
            old_instructions.push_back(old_block.instructions.at(i));
          }
          // truncate old block
          while (old_block.instructions.size() > split_pos)
          {
            old_block.instructions.pop_back();
          }
          // assume (conservatively) that these instructions are used in the same JIT blocks as the
          // ones before the split
          std::vector<u32> inlined_in = old_block.inlined_in;
          inlined_in.push_back(iter->first);

          u64 runcount = old_block.runcount;
          iter = jit_block_db.emplace_hint(
              insert_pos, report_block.address,
              BasicBlock{std::move(old_instructions), runcount, {}, std::move(inlined_in)});
        }
      }
      else
      {
        // new block at lowest observed address
        iter = jit_block_db.emplace_hint(iter, report_block.address, BasicBlock{{}, 0, {}, {}});
      }
      ASSERT(iter->first == report_block.address);
      iter->second.runcount += report_block.usecount;
      std::vector<DecodedInstruction>& insts = iter->second.instructions;
      u32 length = max_length;
      if (report_block.length < length)
      {
        length = report_block.length;
      }
      if (insts.size() < length)
      {
        // new instructions (overrun): just copy everything, maybe we even get new instrumentation
        // data
        INFO_LOG(DYNA_REC, "recording %u instructions @ %08x, was %zu", length,
                 report_block.address, insts.size());
        insts.clear();
        for (u32 i = report_block.offset; i < report_block.offset + length; ++i)
        {
          insts.push_back(baseline_report.instructions.at(i));
        }
        // invalidate JIT blocks
        iter->second.entry_points.clear();
      }
      if (iter->second.runcount > BASELINE_THRESHOLD)
      {
        baseline_suggestions.push_back(report_block.address);
      }
    }
  }

  for (Bail bail : baseline_report.bails)
  {
    all_bails.insert(bail);
  }

  BaselineCompile(baseline_suggestions);

  // the mutex on the report drops here, so next time the CPU thread reports,
  // it will drop the interpreter blocks whose instructions we just received,
  // to be replaced by our JIT code once that address is jumped to next.
  lock.unlock();
  // in the (hopefully unlikely) case that the CPU thread is waiting on us, notify it
  report_cv.notify_one();

  // TODO: analyse instrumentation data, build CFGs/dominator trees, optimize
  return true;
}
