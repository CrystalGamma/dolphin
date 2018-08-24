#include "Core/PowerPC/JitTiered/JitTiered.h"

#include "Core/PowerPC/PowerPC.h"

void JitTiered::ClearCache()
{
  // invalidate all caches
  std::memset(&new_blocks_addrs, 0, sizeof(new_blocks_addrs));
  std::memset(&baseline_report.GetWriter().block_addrs, 0, sizeof(baseline_report.GetWriter().block_addrs));
  std::memset(&dispatch_addrs, 0, sizeof(dispatch_addrs));
  // prevent us from starting any JIT blocks until we are sure Baseline has cleared the block table
  invalidation_mask = BloomAll();
  // signal that everything has been invalidated
  baseline_report.GetWriter().invalidations.push_back({0,0xffffffff});
}

std::vector<JitTiered::DecodedInstruction> &JitTiered::CreateFreeBlock(u32 key, u32 address)
{
  int start = key << INT_CACHE_WAYS_SHIFT;
  // look for invalid entries
  for (int i = 0; i < INT_CACHE_WAYS; i += 1)
  {
    if (new_blocks_addrs[start + i] == 0)
    { // reset block contents
      new_blocks_addrs[start + i] = address;
      new_blocks_instructions[start + i].clear();
      return new_blocks_instructions[start + i];
    }
  }
  int i = new_blocks_clocks[key]; 
  while ((new_blocks_addrs[start + i] & SECOND_CHANCE) == 0)
  {
    new_blocks_addrs[start + i] |= SECOND_CHANCE;
    i = (i + 1) % INT_CACHE_WAYS;
  }
  new_blocks_clocks[key] = i;
  new_blocks_addrs[start + i] = address;
  new_blocks_instructions[start + i].instructions.clear();
  return new_blocks_instructions[start + i];
}

std::optional<int> JitTiered::FindInterpreterBlock(u32 *table, u32 key, u32 address)
{
  FreeBlockHeader *set = new_blocks + (key << INT_CACHE_WAYS_SHIFT);
  for (int i = 0; i < INT_CACHE_WAYS; i += 1)
  {
    if (set[i].start_addr & 0xfffffffc == address)
    {
      return set[i];
    }
  }
  return {};
}

/// interpret one block (does not attempt to call JIT code)
void JitTiered::InterpretBlock()
{
  if (PC == 0)
  {
    // zero is a null value in the cache tags, so handle it specially in case code jumps there
    auto inst = PowerPC::Read_Opcode(0);
    if (inst.hex == 0)
    {
      PowerPC::CheckExceptions();
      return;
    }
    inst.func(inst.inst);
    PowerPC::ppcState.downcount -= inst.cycles;
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
  u32 cache_key = InterpreterCacheKey(start_addr);
  auto free_block_index = FindInterpreterBlock(new_blocks_addrs, cache_key, start_addr);
  std::vector<DecodedInstruction> *free_block;
  u32 cycles;
  if (!free_block_index.has_value())
  { // no free block found, look for compacted block
    auto report = baseline_report.GetWriter();
    auto comp_block = FindInterpreterBlock(report.blocks_addrs, key, PC);
    u32 start = 0, end = 0;
    if (comp_block.has_value())
    {
      end = report.blocks_ends[comp_block];
      if (comp_block != 0)
      {
        start = report.blocks_ends[comp_block - 1];
      }
      for (u32 i = start; i < end; i += 1)
      {
        auto &inst = report.instructions[i];
        inst.func(inst.inst);
        if (PowerPC::ppcState.Exceptions)
        {
          PowerPC::CheckExceptions();
          PowerPC::ppcState.downcount -= inst.cycles;
          return;
        }
        if (NPC != PC + 4)
        {
          PowerPC::ppcState.downcount -= inst.cycles;
          PC = NPC;
          return;
        }
        PC = NPC;
      }
      if (start != end)
      {
        auto &last = report.instructions[end - 1];
        if (InstructionClassifier::Redispatch(last.inst) || PowerPC::breakpoints.IsAddressBreakPoint(PC))
        { // even if the block didn't end here, we have to go back to dispatcher, because of e. g. invalidation or breakpoints
          PowerPC::ppcState.downcount -= inst.cycles;
          return;
        }
        cycles = last.cycles;
      }
    }
    // overran the compacted block (or didn't find one), create free block
    free_block = &CreateFreeBlock(cache_key, start_addr);
    if (free_block->empty())
    {
      for (u32 i = start; i < end; i += 1)
      {
        free_block->push_back(compacted_instructions[i]);
      }
    }
  }
  else
  { // free block found
    free_block = free_block_instructions + free_block_index;
    iter = free_block->begin();
    while (iter != free_block->end())
    {
      auto inst = *iter++;
      inst.func(inst.inst);
      if (PowerPC::ppcState.Exceptions)
      {
        PowerPC::CheckExceptions();
        PowerPC::ppcState.downcount -= inst.cycles;
        return;
      }
      PC += 4;
      if (NPC != PC)
      {
        PowerPC::ppcState.downcount -= inst.cycles;
        PC = NPC;
        return;
      }
      PC = NPC;
    }
    if (free_block->size() > 0)
    {
      auto &last = free_block->back();
      cycles = last.cycles;
      if (InstructionClassifier::Redispatch(last.inst) || PowerPC::breakpoints.IsAddressBreakPoint(PC))
      {
        PowerPC::ppcState.downcount -= cycles;
        return;
      }
    }
  }
  // overrun: read more instructions
  do {
    PowerPC::CheckBreakPoints();
    auto inst = PowerPC::Read_Opcode(PC);
    if (inst.hex == 0)
    {
      PowerPC::CheckExceptions();
      return;
    }
    if (PowerPC::breakpoints.IsAddressBreakPoint(PC))
    {
      break;
    }
    cycles += InstructionClassifier::Cycles(inst);
    auto func = PPCTables::GetInterpreterOp(op.inst)
    free_block->push_back({inst, cycles, func});
    func(inst);
    if (PowerPC::ppcState.Exceptions)
    {
      PowerPC::CheckExceptions();
      PowerPC::ppcState.downcount -= cycles;
      return;
    }
    if (InstructionClassifier::Redispatch(inst))
    {
      break;
    }
    PC += 4;
  } while (PC == NPC);
  PowerPC::ppcState.downcount -= cycles;
  PC = NPC;
}

void JitTiered::SingleStep()
{
  CoreTiming::Advance();
  InterpretBlock();
}

void JitTiered::CompactInterpreterBlocks()
{
  auto report = baseline_report.GetWriter();
  report.instructions.clear();
  std::memcpy(&report.block_addrs, &new_blocks_addrs, sizeof(new_blocks_addrs));
  for (int i = 0; i < INT_CACHE_SIZE; i += 1)
  {
    if (new_blocks_addrs[i] != 0)
    { // invalid blocks are only cleared lazily (for safety as much as simplicity), so don't copy invalid instructions
      for (auto inst : new_blocks_instructions[i])
      {
        report.instructions.push_back(inst);
      }
    }
    report.blocks_ends[i] = compacted_instructions.size();
  }
}

void JitTiered::Run()
{
  const CPU::State* state = CPU::GetStatePtr();
  while (*state == CPU::State::Running)
  {
    CoreTiming::Advance();
    int new_dc = PowerPC::ppcState.downcount;
    do {
      int dc = new_dc;
      InterpretBlock();
      new_dc = PowerPC::ppcState.downcount;
      if ((new_dc >> BASELINE_REPORT_SHIFT) - (dc >> BASELINE_REPORT_SHIFT) > 0)
      {
        auto guard = baseline_report.Yield();
        if (guard.has_value())
        {
          CompactInterpreterBlocks();
          auto copy = baseline_report.GetWriter();
          auto report = guard.GetRef();
          std::memcpy(&report.block_addrs, &copy.blocks_addrs, sizeof(old_blocks_addrs));
          std::memcpy(&report.block_ends, &copy.blocks_ends, sizeof(old_blocks_ends));
        }
      }
    } while (new_dc > 0 && *state == CPU::State::Running);
}
