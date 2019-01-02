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
  report->instructions.clear();
  report->instructions.push_back({});
  report->blocks.clear();
  for (auto& entry : dispatch_cache)
  {
    if (!entry.IsValid())
    {
      continue;
    }
    if (entry.executor == interpreter_executor)
    {
      const u32 offset = entry.offset;
      const u32 len = entry.length;
      if (len > 0 && (keep_old_blocks || offset >= offset_new))
      {
        const u32 pos = u32(report->instructions.size());
        for (u32 n = 0; n < len; n += 1)
        {
          report->instructions.push_back(next_report.instructions[offset + n]);
        }
        report->instructions.push_back({});
        // update offset to position after compaction
        entry.offset = pos;
      }
      else
      {
        // block is old (or empty) and will disappear with this compaction ⇒ invalidate
        DEBUG_LOG(DYNA_REC, "compacting away block @ %08x (used %u times)", entry.address,
                  entry.usecount);
        entry.Invalidate();
      }
    }
    report->blocks.push_back(entry);
    entry.usecount = 0;
  }
  // copy compacted blocks into interpreter cache
  next_report.instructions = report->instructions;
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
  entry->length = 0;
  entry->usecount = 1;
  return entry;
}

bool JitTieredGeneric::IsRedispatchInstruction(const UGeckoInstruction inst)
{
  const GekkoOPInfo* info = PPCTables::GetOpInfo(inst);
  return inst.OPCD == 9                               // sc
         || (inst.OPCD == 31 && inst.SUBOP10 == 146)  // mtmsr
         || (info->flags & FL_CHECKEXCEPTIONS)        // rfi
         || (info->type == OpType::InstructionCache)  // isync
      ;
}

bool JitTieredGeneric::InterpretBlock(const DecodedInstruction* instructions)
{
  for (; instructions->func; ++instructions)
  {
    auto& inst = *instructions;
    NPC = PC + 4;
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
      PowerPC::ppcState.downcount -= inst.cycles;
      return false;
    }
    if (NPC != PC + 4)
    {
      PowerPC::ppcState.downcount -= inst.cycles;
      PC = NPC;
      return false;
    }
    PC = NPC;
  }
  if ((instructions - 1)->func)
  {
    const DecodedInstruction& last = *(instructions - 1);
    PowerPC::ppcState.downcount -= last.cycles;
    if (IsRedispatchInstruction(last.inst))
    {
      // even if the block didn't end here, we have to go back to dispatcher
      // because of e. g. invalidation or breakpoints
      return false;
    }
  }
  return true;
}

void JitTieredGeneric::HandleOverrun(DispatchCacheEntry* cache_entry)
{
  const u32 len = cache_entry->length;
  const u32 offset = cache_entry->offset;
  if (offset + len + 1 != next_report.instructions.size())
  {
    // add sentinel instruction (so InterpretBlock can detect the end of the previous block)
    next_report.instructions.push_back({});
    // we can only append to the last block, so copy this one to the end
    cache_entry->offset = static_cast<u32>(next_report.instructions.size());
    for (u32 pos = offset; pos < offset + len + 1; pos += 1)
    {
      next_report.instructions.push_back(next_report.instructions[pos]);
    }
  }
  u16 cycles_total = 0;
  if (len > 0)
  {
    cycles_total = next_report.instructions[next_report.instructions.size() - 2].cycles;
  }

  u16 cycles_new = 0;
  do
  {
    // if length is 0, it means we are resuming after the breakpoint
    if (PowerPC::breakpoints.IsAddressBreakPoint(PC) && cache_entry->length != 0)
    {
      INFO_LOG(DYNA_REC, "%8x: breakpoint", PC);
      break;
    }
    const UGeckoInstruction inst(PowerPC::Read_Opcode(PC));
    if (inst.hex == 0)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_ISI;
      PowerPC::ppcState.downcount -= cycles_new;
      return;
    }
    u16 inst_cycles = PPCTables::GetOpInfo(inst)->numCycles;
    if (inst_cycles > 0xffff - cycles_total)
    {
      // prevent timing overflow by ending the (very large) block early (deterministically!)
      break;
    }
    cycles_new += inst_cycles;
    cycles_total += inst_cycles;
    const InterpreterFunc func = PPCTables::GetInterpreterOp(inst);
    const bool uses_fpu = PPCTables::UsesFPU(inst);
    DecodedInstruction dec_inst;
    dec_inst.func = func;
    dec_inst.inst = inst;
    dec_inst.cycles = cycles_total;
    dec_inst.flags = uses_fpu ? USES_FPU : 0;
    cache_entry->length += 1;
    if ((PC & 31) == 0)
    {
      cache_entry->bloom |= BloomCacheline(PC);
    }
    NPC = PC + 4;
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
      PowerPC::ppcState.downcount -= cycles_new;
      return;
    }
    PC += 4;
    if (IsRedispatchInstruction(inst))
    {
      break;
    }
  } while (PC == NPC);
  PowerPC::ppcState.downcount -= cycles_new;
  PC = NPC;
}

void JitTieredGeneric::SingleStep()
{
  CoreTiming::Advance();
  PowerPC::CheckExternalExceptions();
  DispatchCacheEntry* entry = FindBlock(PC);
  entry->usecount += 1;
  u32 flags = entry->executor(this, entry->offset, &PowerPC::ppcState, nullptr);
  if (flags & BLOCK_OVERRUN)
  {
    HandleOverrun(entry);
  }
  PowerPC::CheckExceptions();
  PowerPC::CheckBreakPoints();
}

void JitTieredGeneric::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  BaselineReport last_report;
  while (*state == CPU::State::Running)
  {
    CoreTiming::Advance();
    PowerPC::CheckExternalExceptions();
    // this heuristic works well for the interpreter-only case
    size_t num_instructions = next_report.instructions.size();
    if (num_instructions >= (1 << 20) && num_instructions >= 2 * last_report.instructions.size())
    {
      CompactInterpreterBlocks(&last_report, true);
      next_report.invalidations.clear();
    }
    do
    {
      DispatchCacheEntry* entry = FindBlock(PC);
      entry->usecount += 1;
      u32 flags = entry->executor(this, entry->offset, &PowerPC::ppcState, nullptr);
      if (flags & BLOCK_OVERRUN)
      {
        HandleOverrun(entry);
      }
      PowerPC::CheckExceptions();
      PowerPC::CheckBreakPoints();
    } while (PowerPC::ppcState.downcount > 0 && *state == CPU::State::Running);
  }
}
