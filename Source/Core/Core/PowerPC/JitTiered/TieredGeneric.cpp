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
  std::memset(&disp_cache, 0, sizeof(disp_cache));
}

static bool overlaps(u32 block_start, u32 block_len, u32 first_inval, u32 last_inval)
{
  return (block_start <= last_inval &&
          (block_start + block_len) >
              first_inval)  // (block_start + block_len) is *exclusive* end of the block interval
         || (block_start >= first_inval && block_start <= last_inval);
}

void JitTieredGeneric::InvalidateICache(u32 address, u32 size, bool forced)
{
  if (size == 0)
  {
    return;
  }
  u32 end = address + size - 1;
  for (size_t i = 0; i < DISP_CACHE_SIZE; i += 1)
  {
    if ((disp_cache[i].address & 3) == 0)
    {
      // interpreter block ⇒ 'range' is length of the block
      // catches 0 case (invalid entry) too, but this invalidates anyway
      if (overlaps(disp_cache[i].address & FLAG_MASK, disp_cache[i].len * 4, address, end))
      {
        disp_cache[i].address = 0;
      }
    }
  }
}

void JitTieredGeneric::CompactInterpreterBlocks()
{
  auto report = baseline_report.GetWriter();
  report.instructions.clear();
  report.blocks.clear();
  for (size_t i = 0; i < DISP_CACHE_SIZE; i += 1)
  {
    u32 address = disp_cache[i].address;
    if ((address & 3) == 0 && address)
    {
      u32 offset = disp_cache[i].offset;
      if (offset >= offset_new)
      {
        u32 start = static_cast<u32>(report.instructions.size());
        report.blocks.push_back({address, start, disp_cache[i].usecount});
        u32 len = static_cast<u32>(disp_cache[i].len);
        for (u32 n = 0; n < len; n += 1)
        {
          report.instructions.push_back(inst_cache[offset + n]);
        }
        // update offset to position after compaction
        disp_cache[i].offset = start;
      }
      else
      {
        // block is old and will disappear with this compaction ⇒ invalidate
        disp_cache[i].address = 0;
      }
    }
  }
  // copy compacted blocks into interpreter cache
  inst_cache = report.instructions;
  offset_new = report.instructions.size();
  // sentinel value to make calculating the length in Baseline easier
  report.blocks.push_back({0, static_cast<u32>(offset_new), 0});
}

static bool IsRedispatchInstruction(UGeckoInstruction inst)
{
  auto info = PPCTables::GetOpInfo(inst);
  return inst.OPCD == 9                               // sc
         || (inst.OPCD == 31 && inst.SUBOP10 == 146)  // mtmsr
         || (info->flags & FL_CHECKEXCEPTIONS)        // rfi
         || (info->type == OpType::InstructionCache)  // isync
      //|| (info->type == OpType::Branch) // isync
      ;
}

void JitTieredGeneric::InterpretBlock()
{
  if (PC == 0)
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
  u32 start_addr = PC;
  u32 key = DispatchCacheKey(start_addr);
  if ((disp_cache[key].address & FLAG_MASK) != start_addr)
  {
    // not in cache: create new Interpreter block (or look up in JIT block DB once that's
    // implemented)
    disp_cache[key].address = start_addr;
    disp_cache[key].offset = (u32)inst_cache.size();
    disp_cache[key].len = disp_cache[key].usecount = 0;
  }
  if (disp_cache[key].address == start_addr)
  {
    // flag bits are zero: interpreter block
    u32 len = disp_cache[key].len;
    disp_cache[key].usecount += 1;
    u32 offset = disp_cache[key].offset;
    for (u32 pos = offset; pos < offset + len; pos += 1)
    {
      auto& inst = inst_cache[pos];
      NPC = PC + 4;
      if (inst.uses_fpu && !MSR.FP)
      {
        INFO_LOG(DYNA_REC, "%8x: fresh block FP exception exit", PC);
        PowerPC::ppcState.Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
        PowerPC::CheckExceptions();
        PowerPC::ppcState.downcount -= inst.cycles;
        PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
        return;
      }
      inst.func(inst.inst);
      if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
      {
        INFO_LOG(DYNA_REC, "%8x: cached block exception exit", PC);
        PowerPC::CheckExceptions();
        PowerPC::ppcState.downcount -= inst.cycles;
        PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
        return;
      }
      if (NPC != PC + 4)
      {
        INFO_LOG(DYNA_REC, "%8x: cached block finished (downcount %d)", PC,
                 PowerPC::ppcState.downcount);
        PowerPC::ppcState.downcount -= inst.cycles;
        PC = NPC;
        return;
      }
      PC = NPC;
    }
    u32 cycles = 0;
    if (len != 0)
    {
      auto& last = inst_cache[offset + len - 1];
      cycles = last.cycles;
      if (IsRedispatchInstruction(last.inst) || PowerPC::breakpoints.IsAddressBreakPoint(PC))
      {  // even if the block didn't end here, we have to go back to dispatcher, because of e. g.
         // invalidation or breakpoints
        INFO_LOG(DYNA_REC, "%8x: hit context sync or breakpoint", PC);
        PowerPC::ppcState.downcount -= cycles;
        return;
      }
    }
    // overrun
    if (offset + len != inst_cache.size())
    {
      // we can only append to the last block, so copy this one to the end
      disp_cache[key].offset = inst_cache.size();
      for (u32 pos = offset; pos < offset + len; pos += 1)
      {
        inst_cache.push_back(inst_cache[pos]);
      }
      // following code should not index into inst_cache, but just to be sure
      offset = disp_cache[key].offset;
    }
    do
    {
      // handle temporary breakpoints
      PowerPC::CheckBreakPoints();
      auto inst = UGeckoInstruction(PowerPC::Read_Opcode(PC));
      if (inst.hex == 0)
      {
        INFO_LOG(DYNA_REC, "%8x: instruction fetch exception", PC);
        PowerPC::ppcState.Exceptions |= EXCEPTION_ISI;
        PowerPC::CheckExceptions();
        return;
      }
      // end the block only on permanent breakpoints
      if (PowerPC::breakpoints.IsAddressBreakPoint(PC))
      {
        INFO_LOG(DYNA_REC, "%8x: breakpoint", PC);
        break;
      }
      cycles += PPCTables::GetOpInfo(inst)->numCycles;
      auto func = PPCTables::GetInterpreterOp(inst);
      DecodedInstruction dec_inst = {func, inst, cycles,
                                     static_cast<u32>(PPCTables::UsesFPU(inst))};
      inst_cache.push_back(dec_inst);
      disp_cache[key].len += 1;
      NPC = PC + 4;
      if (dec_inst.uses_fpu && !MSR.FP)
      {
        INFO_LOG(DYNA_REC, "%8x: fresh block FP exception exit", PC);
        PowerPC::ppcState.Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
        PowerPC::CheckExceptions();
        PowerPC::ppcState.downcount -= cycles;
        PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
        return;
      }
      func(inst);
      if (PowerPC::ppcState.Exceptions & EXCEPTION_SYNC)
      {
        INFO_LOG(DYNA_REC, "%8x: fresh block exception exit", PC);
        PowerPC::CheckExceptions();
        PowerPC::ppcState.downcount -= cycles;
        PowerPC::ppcState.Exceptions &= ~EXCEPTION_SYNC;
        return;
      }
      PC += 4;
      if (IsRedispatchInstruction(inst))
      {
        break;
      }
    } while (PC == NPC);
    PowerPC::ppcState.downcount -= cycles;
    PC = NPC;
  }
}

void JitTieredGeneric::SingleStep()
{
  CoreTiming::Advance();
  InterpretBlock();
}

void JitTieredGeneric::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  while (*state == CPU::State::Running)
  {
    /*int jit_throttle = JIT_THROTTLE;
    jit_throttle -= PowerPC::ppcState.downcount;*/
    CoreTiming::Advance();
    /*jit_throttle -= PowerPC::ppcState.downcount;*/
    if (/*jit_throttle < 0*/ inst_cache.size() > (1 << 16))
    {
      // this will switch sides on the Baseline report, causing compaction
      baseline_report.Wait();
    }
    auto guard = baseline_report.Yield();
    if (guard.has_value())
    {
      CompactInterpreterBlocks();
    }
    do
    {
      InterpretBlock();
    } while (PowerPC::ppcState.downcount > 0 && *state == CPU::State::Running);
  }
}