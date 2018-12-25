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

static constexpr u32 EXCEPTION_SYNC =
    ~(EXCEPTION_EXTERNAL_INT | EXCEPTION_PERFORMANCE_MONITOR | EXCEPTION_DECREMENTER);

void JitTieredGeneric::ClearCache()
{
  // invalidate dispatch cache
  disp_cache = {};
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
  // INFO_LOG(DYNA_REC, "%08x: invalidate %08x–%08x", PC, address, end);
  next_report.invalidation_bloom |= BloomRange(address, end);
  next_report.invalidations.push_back({address, end});
  for (auto& entry : disp_cache)
  {
    if ((entry.address & FLAG_MASK) <= 1)
    {
      // catches 0 case (invalid entry) too, but this invalidates anyway (⇒ safe)
      if (overlaps(entry.address & ~FLAG_MASK, entry.len * 4, address, end))
      {
        INFO_LOG(DYNA_REC, "removing Baseline block @ %08x", entry.address & ~FLAG_MASK);
        entry.address = 0;
      }
    }
    else if (entry.bloom & next_report.invalidation_bloom)
    {
      INFO_LOG(DYNA_REC, "removing optimized block @ %08x", entry.address & ~FLAG_MASK);
      entry.address = 0;
    }
  }
}

void JitTieredGeneric::CompactInterpreterBlocks(BaselineReport* const report,
                                                const bool keep_old_blocks)
{
  report->instructions.clear();
  report->blocks.clear();
  u32 pos = 0;
  for (auto& entry : disp_cache)
  {
    const u32 address = entry.address;
    if (!address)
    {
      continue;
    }
    if ((address & FLAG_MASK) <= 1)
    {
      report->blocks.push_back({address, pos, entry.usecount});
      if ((address & FLAG_MASK) == 0)
      {
        const u32 offset = entry.offset;
        const u32 len = static_cast<u32>(entry.len);
        if (len > 0 && (keep_old_blocks || offset >= offset_new))
        {
          for (u32 n = 0; n < len; n += 1)
          {
            report->instructions.push_back(next_report.instructions[offset + n]);
          }
          // update offset to position after compaction
          entry.offset = pos;
          pos += len;
        }
        else
        {
          // block is old (or empty) and will disappear with this compaction ⇒ invalidate
          INFO_LOG(DYNA_REC, "compacting away block @ %08x (used %u times)",
                   entry.address & ~FLAG_MASK, entry.usecount);
          entry.address = 0;
        }
      }
    }
  }
  // copy compacted blocks into interpreter cache
  next_report.instructions = report->instructions;
  offset_new = pos;
  // sentinel value to make calculating the length in Baseline easier
  report->blocks.push_back({0, static_cast<u32>(offset_new), 0});
}

u32 JitTieredGeneric::FindBlock(const u32 address)
{
  const u32 key = DispatchCacheKey(address);
  DispCacheEntry& primary_entry = disp_cache.at(key);
  const u32 cache_addr = primary_entry.address;
  if ((cache_addr & ~FLAG_MASK) == address)
  {
    // best outcome: we have the block in primary cache
    return key;
  }
  // not in primary cache, search victim cache
  const u32 victim_set = key >> (DISP_CACHE_SHIFT - VICTIM_SETS_SHIFT);
  const u32 victim_start = DISP_PRIMARY_CACHE_SIZE + (victim_set << VICTIM_WAYS_SHIFT);
  const u32 victim_end = victim_start + VICTIM_WAYS;
  bool found_vacancy = false;
  u32 pos = 0;
  for (u32 i = victim_start; i < victim_end; i += 1)
  {
    DispCacheEntry& victim_entry = disp_cache.at(i);
    if ((victim_entry.address & ~FLAG_MASK) == address)
    {
      // second-best outcome: we find the block in victim cache ⇒ swap and done
      // INFO_LOG(DYNA_REC, "%08x: victim hit", PC);
      DispCacheEntry tmp = primary_entry;
      primary_entry = victim_entry;
      victim_entry = tmp;
      victim_second_chance.set(i - DISP_PRIMARY_CACHE_SIZE);
      return key;
    }
    if (victim_entry.address == 0)
    {
      // third-best outcome: we have a vacancy in victim cache
      // (but continue searching for a real match)
      found_vacancy = true;
      pos = i;
    }
  }
  if (cache_addr != 0)
  {
    if (found_vacancy)
    {
      // INFO_LOG(DYNA_REC, "%08x: swap %08x", PC, cache_addr);
      disp_cache.at(pos) = primary_entry;
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
      WARN_LOG(DYNA_REC, "%08x: evict %08x", PC, disp_cache.at(pos).address);
      victim_clocks.at(victim_set) = clock;
      disp_cache.at(pos) = primary_entry;
      victim_second_chance.set(pos - DISP_PRIMARY_CACHE_SIZE);
    }
  }
  // at this point, the primary cache contains an invalid entry
  // (or one that has already been saved to victim cache)

  // let the subclass decide how to get a block
  // (or whether to create an interpreter block instead)
  return LookupBlock(key, address);
}

u32 JitTieredGeneric::LookupBlock(const u32 key, const u32 address)
{
  // we have no JIT to get blocks from, so just create new interpreter block unconditionally
  INFO_LOG(DYNA_REC, "%08x: new block", PC);
  auto& entry = disp_cache.at(key);
  entry.address = address;
  entry.offset = static_cast<u32>(next_report.instructions.size());
  entry.len = entry.usecount = 0;
  return key;
}

static bool IsRedispatchInstruction(const UGeckoInstruction inst)
{
  const GekkoOPInfo* info = PPCTables::GetOpInfo(inst);
  return inst.OPCD == 9                               // sc
         || (inst.OPCD == 31 && inst.SUBOP10 == 146)  // mtmsr
         || (info->flags & FL_CHECKEXCEPTIONS)        // rfi
         || (info->type == OpType::InstructionCache)  // isync
      ;
}

void JitTieredGeneric::RunZeroInstruction()
{
  while (PC == 0)
  {
    // zero is a null value in the cache tags, so handle it specially in case code jumps there
    u32 inst = PowerPC::Read_Opcode(0);
    if (inst == 0)
    {
      PowerPC::CheckExceptions();
      return;
    }
    PPCTables::GetInterpreterOp(UGeckoInstruction(inst))(inst);
    PowerPC::ppcState.downcount -= PPCTables::GetOpInfo(inst)->numCycles;
    if (PowerPC::ppcState.downcount < 0)
    {
      CoreTiming::Advance();
    }
    if (PowerPC::ppcState.Exceptions)
    {
      PowerPC::CheckExceptions();
      return;
    }
    PC = NPC;
    if (PC != 4)
    {
      return;
    }
  }
}

bool JitTieredGeneric::InterpretBlock(u32 index)
{
  const u32 len = disp_cache.at(index).len;
  const u32 offset = disp_cache.at(index).offset;
  for (u32 pos = offset; pos < offset + len; pos += 1)
  {
    auto& inst = next_report.instructions[pos];
    NPC = PC + 4;
    if (inst.uses_fpu && !MSR.FP)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
    }
    else
    {
      inst.func(inst.inst);
    }
    if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
    {
      PowerPC::CheckExceptions();
      PowerPC::ppcState.downcount -= inst.cycles;
      PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
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
  if (len != 0)
  {
    const DecodedInstruction& last = next_report.instructions[offset + len - 1];
    PowerPC::ppcState.downcount -= last.cycles;
    if (IsRedispatchInstruction(last.inst) || PowerPC::breakpoints.IsAddressBreakPoint(PC))
    {
      // even if the block didn't end here, we have to go back to dispatcher
      // because of e. g. invalidation or breakpoints
      return false;
    }
  }
  return true;
}

void JitTieredGeneric::ReadInstructions(u32 index)
{
  auto& cache_entry = disp_cache.at(index);
  const u32 len = cache_entry.len;
  const u32 offset = cache_entry.offset;
  u32 cycles_old = 0;
  if (offset + len != next_report.instructions.size())
  {
    // we can only append to the last block, so copy this one to the end
    cache_entry.offset = static_cast<u32>(next_report.instructions.size());
    for (u32 pos = offset; pos < offset + len; pos += 1)
    {
      next_report.instructions.push_back(next_report.instructions[pos]);
      cycles_old = next_report.instructions[pos].cycles;
    }
    // following code must not index into next_report.instructions
  }
  u32 cycles_new = 0;
  do
  {
    // handle temporary breakpoints
    PowerPC::CheckBreakPoints();
    const UGeckoInstruction inst(PowerPC::Read_Opcode(PC));
    if (inst.hex == 0)
    {
      PowerPC::ppcState.Exceptions |= EXCEPTION_ISI;
      PowerPC::ppcState.downcount -= cycles_new;
      PowerPC::CheckExceptions();
      return;
    }
    // end the block only on permanent breakpoints
    if (PowerPC::breakpoints.IsAddressBreakPoint(PC))
    {
      INFO_LOG(DYNA_REC, "%8x: breakpoint", PC);
      break;
    }
    cycles_new += PPCTables::GetOpInfo(inst)->numCycles;
    const InterpreterFunc func = PPCTables::GetInterpreterOp(inst);
    const bool uses_fpu = PPCTables::UsesFPU(inst);
    DecodedInstruction dec_inst;
    dec_inst.func = func;
    dec_inst.inst = inst;
    dec_inst.cycles = cycles_old + cycles_new;
    dec_inst.uses_fpu = static_cast<u32>(uses_fpu);
    dec_inst.needs_slowmem = 1;
    cache_entry.len += 1;
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
        u32 addr =
            static_cast<u32>(static_cast<s64>(base) + (static_cast<s64>(1) << 32) + inst.SIMM_16);
        dec_inst.needs_slowmem = static_cast<u32>(!PowerPC::IsOptimizableRAMAddress(addr));
      }
      func(inst);
    }
    next_report.instructions.push_back(dec_inst);
    if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
    {
      PowerPC::ppcState.downcount -= cycles_new;
      PowerPC::CheckExceptions();
      PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
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
  RunZeroInstruction();
  u32 key = FindBlock(PC);
  if ((disp_cache.at(key).address & FLAG_MASK) <= 1)
  {
    disp_cache.at(key).usecount += 1;
  }
  InterpretBlock(key);
}

void JitTieredGeneric::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  BaselineReport last_report;
  while (*state == CPU::State::Running)
  {
    CoreTiming::Advance();
    // this heuristic works well for the interpreter-only case
    size_t num_instructions = next_report.instructions.size();
    if (num_instructions >= (1 << 20) && num_instructions >= 2 * last_report.instructions.size())
    {
      CompactInterpreterBlocks(&last_report, true);
      next_report.invalidations.clear();
    }
    do
    {
      RunZeroInstruction();
      u32 start_addr = PC;
      u32 key = FindBlock(start_addr);
      // the generic path only creates interpreter blocks.
      disp_cache.at(key).usecount += 1;
      bool overrun = InterpretBlock(key);
      if (overrun)
      {
        ReadInstructions(key);
      }
    } while (PowerPC::ppcState.downcount > 0 && *state == CPU::State::Running);
  }
}
