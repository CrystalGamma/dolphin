// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/JitTiered.h"

#include "Common/Logging/Log.h"
#include "Core/CoreTiming.h"
#include "Core/HW/CPU.h"
#include "Core/PowerPC/Interpreter/Interpreter.h"
#include "Core/PowerPC/MMU.h"
#include "Core/PowerPC/PowerPC.h"

void JitTieredGeneric::ClearCache()
{
  // invalidate dispatch cache
  dispatch_cache = {};
  next_report.invalidation_bloom = BloomAll();
  next_report.invalidations.push_back({0, 0xffffffff});
  INFO_LOG(DYNA_REC, "%08x: clearing cache", PC);
}

static bool overlaps(u32 block_start, u32 block_len, u32 first_inval, u32 last_inval)
{
  // (block_start + block_len) is *exclusive* end of the block interval
  return (block_start <= last_inval && (block_start + block_len) > first_inval) ||
         (block_start >= first_inval && block_start <= last_inval);
}

void JitTieredGeneric::InvalidateICache(const u32 address, const u32 size, const bool forced)
{
  // fast path for guest cacheline invalidations
  if (size == 32)
  {
    if (!valid_block.Test(address >> 5))
    {
      return;
    }
    valid_block.Clear(address >> 5);
  }
  if (size == 0)
  {
    return;  // zero-sized invalidations don't have a 'last invalidated byte' so exit early
  }

  const u32 end = address + size - 1;
  DEBUG_LOG(DYNA_REC, "%08x: invalidate %08x–%08x", PC, address, end);
  Bloom new_bloom = BloomRange(address, end);
  next_report.invalidation_bloom |= new_bloom;
  next_report.invalidations.push_back({address, end});
  for (auto& entry : dispatch_cache)
  {
    if (!(entry.bloom & new_bloom))
    {
      continue;
    }
    if (entry.length != 0)
    {
      if (overlaps(entry.address, entry.length * 4, address, end))
      {
        DEBUG_LOG(DYNA_REC, "removing simple block @ %08x", entry.address);
        entry.Invalidate();
      }
    }
    else
    {
      DEBUG_LOG(DYNA_REC, "removing optimized block @ %08x", entry.address);
      entry.Invalidate();
    }
  }
}

void JitTieredGeneric::CompactInterpreterBlocks(BaselineReport* const report,
                                                const bool keep_old_blocks)
{
  // swap all instructions into the report
  next_report.instructions.swap(report->instructions);
  next_report.instructions.clear();
  next_report.instructions.push_back({});
  report->blocks.clear();
  for (auto& entry : dispatch_cache)
  {
    if (!entry.IsValid())
    {
      continue;
    }
    report->blocks.push_back(entry);
    if (entry.executor == interpreter_executor)
    {
      const u32 offset = entry.offset;
      const u32 len = entry.length;
      if (len > 0 && (keep_old_blocks || offset >= offset_new))
      {
        // update offset to position after compaction
        entry.offset = next_report.instructions.size();
        // copy recent interpreter block instructions back
        for (u32 n = 0; n < len; n += 1)
        {
          next_report.instructions.push_back(report->instructions[offset + n]);
        }
        next_report.instructions.push_back({});
      }
      else
      {
        // block is old (or empty) and will disappear with this compaction ⇒ invalidate
        DEBUG_LOG(DYNA_REC, "compacting away block @ %08x (used %u times)", entry.address,
                  entry.usecount);
        entry.Invalidate();
      }
    }
    entry.usecount = 0;
  }
  offset_new = next_report.instructions.size();
}

JitTieredGeneric::DispatchCacheEntry* JitTieredGeneric::FindBlock(const u32 address)
{
  const u32 key = DispatchCacheKey(address);
  DispatchCacheEntry& primary_entry = dispatch_cache.at(key);
  if (primary_entry.address == address && primary_entry.IsValid())
  {
    // best outcome: we have the block in primary cache
    return &primary_entry;
  }
  // not in primary cache, search victim cache
  const u32 victim_set = key >> (DISP_CACHE_SHIFT - VICTIM_SETS_SHIFT);
  const u32 victim_start = DISP_PRIMARY_CACHE_SIZE + (victim_set << VICTIM_WAYS_SHIFT);
  const u32 victim_end = victim_start + VICTIM_WAYS;
  bool found_vacancy = false;
  u32 pos = 0;
  for (u32 i = victim_start; i < victim_end; i += 1)
  {
    DispatchCacheEntry& victim_entry = dispatch_cache.at(i);
    if (!victim_entry.IsValid())
    {
      // third-best outcome: we have a vacancy in victim cache
      // (but continue searching for a real match)
      found_vacancy = true;
      pos = i;
    }
    else if (victim_entry.address == address)
    {
      // second-best outcome: we find the block in victim cache ⇒ swap and done
      DispatchCacheEntry tmp = primary_entry;
      primary_entry = victim_entry;
      victim_entry = tmp;
      victim_second_chance.set(i - DISP_PRIMARY_CACHE_SIZE);
      return &primary_entry;
    }
  }
  if (primary_entry.IsValid())
  {
    if (found_vacancy)
    {
      dispatch_cache.at(pos) = primary_entry;
    }
    else
    {
      // worst outcome: we have to evict from victim cache
      u32 clock = victim_clocks.at(victim_set);
      for (u32 i = 0; i < VICTIM_WAYS; i += 1)
      {
        pos = victim_start + clock;
        if (victim_second_chance.test(pos - DISP_PRIMARY_CACHE_SIZE))
        {
          victim_second_chance.reset(pos - DISP_PRIMARY_CACHE_SIZE);
        }
        else
        {
          break;
        }
        clock = (clock + 1) % VICTIM_WAYS;
      }
      INFO_LOG(DYNA_REC, "%08x: evict %08x", PC, dispatch_cache.at(pos).address);
      victim_clocks.at(victim_set) = clock;
      dispatch_cache.at(pos) = primary_entry;
      victim_second_chance.set(pos - DISP_PRIMARY_CACHE_SIZE);
    }
  }
  // at this point, the primary cache contains an invalid entry
  // (or one that has already been saved to victim cache)

  // let the subclass decide how to get a block
  // (or whether to create an interpreter block instead)
  return LookupBlock(&primary_entry, address);
}

JitTieredGeneric::DispatchCacheEntry* JitTieredGeneric::LookupBlock(DispatchCacheEntry* entry,
                                                                    const u32 address)
{
  // we have no JIT to get blocks from, so just create new interpreter block unconditionally
  DEBUG_LOG(DYNA_REC, "%08x: new block", address);
  entry->address = address;
  if (next_report.instructions.empty())
  {
    next_report.instructions.push_back({});
  }
  entry->offset = static_cast<u32>(next_report.instructions.size());
  next_report.instructions.push_back({});
  entry->executor = interpreter_executor;
  entry->bloom = BloomCacheline(address);
  valid_block.Set(address >> 5);
  entry->length = entry->usecount = 0;
  return entry;
}

bool JitTieredGeneric::IsRedispatchInstruction(const UGeckoInstruction inst)
{
  const GekkoOPInfo* info = PPCTables::GetOpInfo(inst);
  return (info->flags & FL_ENDBLOCK) || (info->type == OpType::InstructionCache)  // isync
      ;
}

u32 JitTieredGeneric::InterpretBlock(const DecodedInstruction* instructions)
{
  for (; instructions->func; ++instructions)
  {
    auto& inst = *instructions;
    // we've already fetched the instruction, so no ISI possible
    NPC = PC + 4;
    // decrementing before executing the instruction (at least logically, in the case of JIT code)
    // is the only way to be deterministic without tracking where the block began and the
    // instructions since then (difficult/slow in the JIT case)
    PowerPC::ppcState.downcount -= inst.cycles;
    if ((inst.flags & USES_FPU) && !MSR.FP)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
    }
    else
    {
      inst.func(inst.inst);
    }
    if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
    {
      return JUMP_OUT;
    }
    if (NPC != PC + 4)
    {
      PC = NPC;
      return JUMP_OUT;
    }
    PC = NPC;
  }
  if ((instructions - 1)->func)
  {
    const DecodedInstruction& last = *(instructions - 1);
    if (IsRedispatchInstruction(last.inst))
    {
      // block ended, but fall through to next instruction ⇒ no downcount/exception check nor
      // overrun
      return 0;
    }
  }
  // this should only be reached if the previous instruction caused an exception every time it was
  // executed before, or the block is empty
  return BLOCK_OVERRUN;
}

bool JitTieredGeneric::HandleOverrun(DispatchCacheEntry* cache_entry)
{
  const u32 len = cache_entry->length;
  if (len == MAX_BLOCK_LENGTH)
  {
    return false;
  }
  const u32 offset = cache_entry->offset;
  if (offset + len + 1 != next_report.instructions.size())
  {
    // we can only append to the last block, so copy this one to the end
    cache_entry->offset = static_cast<u32>(next_report.instructions.size());
    for (u32 pos = offset; pos < offset + len + 1; ++pos)
    {
      next_report.instructions.push_back(next_report.instructions[pos]);
    }
  }

  while (true)
  {
    // if length is 0, it means we are resuming after the breakpoint
    if (PowerPC::breakpoints.IsAddressBreakPoint(PC) && cache_entry->length != 0)
    {
      INFO_LOG(DYNA_REC, "%8x: breakpoint", PC);
      return false;
    }
    const UGeckoInstruction inst(PowerPC::Read_Opcode(PC));
    if (inst.hex == 0)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_ISI;
      return true;
    }

    u16 inst_cycles = PPCTables::GetOpInfo(inst)->numCycles;
    const auto func = PPCTables::GetInterpreterOp(inst);
    const bool uses_fpu = PPCTables::UsesFPU(inst);
    DecodedInstruction dec_inst;
    dec_inst.func = func;
    dec_inst.inst = inst;
    dec_inst.cycles = inst_cycles;
    dec_inst.flags = uses_fpu ? USES_FPU : 0;
    cache_entry->length += 1;
    if ((PC & 31) == 0)
    {
      cache_entry->bloom |= BloomCacheline(PC);
      valid_block.Set(PC >> 5);
    }

    NPC = PC + 4;
    PowerPC::ppcState.downcount -= inst_cycles;

    if (uses_fpu && !MSR.FP)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
    }
    else
    {
      if (inst.OPCD >= 32 && inst.OPCD <= 55 && inst.OPCD != 46 && inst.OPCD != 47)
      {
        // instruction is a non-indexed load/store (but not a multi-word store).
        // we assume that if these are used for MMIO,
        // they will likely access non-RAM addresses the first time.
        // it can save us a lot of backpatching pain
        // to just detect most of them before they are JITed.
        u32 base = inst.RA == 0 ? 0 : GPR(inst.RA);
        u32 addr = u32(s64(base) + (s64(1) << 32) + inst.SIMM_16);
        if (!PowerPC::IsOptimizableRAMAddress(addr))
        {
          dec_inst.flags |= NEEDS_SLOWMEM;
        }
      }
      func(inst);
    }
    next_report.instructions.back() = dec_inst;
    next_report.instructions.push_back({});
    if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
    {
      return true;
    }
    bool jump = NPC != PC + 4;
    PC = NPC;
    if (jump || IsRedispatchInstruction(inst) || cache_entry->length == MAX_BLOCK_LENGTH)
    {
      return jump;
    }
  }
}

void JitTieredGeneric::SingleStep()
{
  CoreTiming::Advance();
  PowerPC::CheckExternalExceptions();
  DispatchCacheEntry* entry = FindBlock(PC);
  entry->usecount += 1;
  u32 flags = entry->executor(this, entry->offset, &PowerPC::ppcState, nullptr);
  bool jump_out = flags & JUMP_OUT;
  if (flags & BLOCK_OVERRUN)
  {
    jump_out = HandleOverrun(entry);
  }
  if (jump_out)
  {
    PowerPC::CheckExceptions();
    // we end after this block anyway, so no downcount check needed
  }
  PowerPC::CheckBreakPoints();
}

void JitTieredGeneric::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  BaselineReport last_report;
  CoreTiming::Advance();
  while (*state == CPU::State::Running)
  {
    DispatchCacheEntry* entry = FindBlock(PC);
    entry->usecount += 1;
    const u32 flags = entry->executor(this, entry->offset, &PowerPC::ppcState, nullptr);
    bool jump_out = flags & JUMP_OUT;
    if (flags & BLOCK_OVERRUN)
    {
      jump_out = HandleOverrun(entry);
    }
    if (jump_out)
    {
      if (PowerPC::ppcState.Exceptions)
      {
        PowerPC::CheckExceptions();
      }
      if (PowerPC::ppcState.downcount <= 0)
      {
        CoreTiming::Advance();
        PowerPC::CheckExternalExceptions();
        // this heuristic works well for the interpreter-only case
        size_t num_instructions = next_report.instructions.size();
        if (num_instructions >= (1 << 20) &&
            num_instructions >= 2 * last_report.instructions.size())
        {
          CompactInterpreterBlocks(&last_report, true);
          next_report.invalidations.clear();
        }
      }
    }
    PowerPC::CheckBreakPoints();
  }
}
