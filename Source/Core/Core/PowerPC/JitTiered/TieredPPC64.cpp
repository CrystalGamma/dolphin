// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/TieredPPC64.h"

#include "Core/PowerPC/PPC64Baseline.h"

JitTieredPPC64::JitTieredPPC64()
{
  codespace.AllocCodeSpace(CODESPACE_CELL_SIZE * CODESPACE_CELLS * 4);
  enter_baseline_block = codespace.GetCodePtr();
  PPCEmitter emit;
  // call frame allocation
  emit.MFSPR(R0, LR);
  emit.STD(R0, R1, 16);
  emit.STD(R1, R1, -32, UPDATE);
  // calculate address
  emit.B(4, true);
  emit.MFSPR(R0, LR);
  emit.RLDICR(R4, R4, 2, 61);
  emit.ADD(R4, R4, R0);
  // jump to address
  emit.MTSPR(CTR, R4);
  emit.BCCTR(BRANCH_ALWAYS, 0);
  codespace.Emit(emit.instructions);
  entry_length = emit.instructions.size();
}

u32 JitTieredPPC64::LookupBlock(u32 key, u32 address)
{
  JitTieredCommon::LookupBlock(key, address);
  if ((disp_cache[key].address & FLAG_MASK) != 0)
  {
    asm volatile("isync");
  }
  return key;
}

void JitTieredPPC64::BaselineCompile(u32 address, JitBlock&& block)
{
  PPC64BaselineCompiler compiler;
  std::vector<u32> instructions;
  for (const DecodedInstruction& inst : block.instructions)
  {
    instructions.push_back(inst.inst);
  }
  compiler.compile(instructions);
  u32 len = block.instructions.size();
  if (len > CODESPACE_CELL_SIZE - entry_length)
  {
    WARN_LOG(DYNA_REC, "Huge block (%u instructions) detected. Refusing to write to code space.",
             len);
    return;
  }
  u32 current_offset = u32(codespace.GetOffset());
  const u32 current_cell = current_offset / CODESPACE_CELL_SIZE;
  const u32 next_cell = ((current_offset + len + 1) / CODESPACE_CELL_SIZE) % CODESPACE_CELLS;
  if (current_cell != next_cell)
  {
    u32 cell_end;
    if (next_cell == 0)
    {
      current_offset = entry_length;
      cell_end = CODESPACE_CELL_SIZE;
    }
    else
    {
      current_offset = next_cell * CODESPACE_CELL_SIZE;
      cell_end = current_offset + CODESPACE_CELL_SIZE;
    }
    WARN_LOG(DYNA_REC, "reclaiming Baseline code space cell %u (%x – %x)", next_cell,
             current_offset, cell_end);
    std::unique_lock lock(block_db_mutex);
    auto iter = jit_block_db.cbegin();
    while (iter != jit_block_db.cend())
    {
      u32 block_offset = iter->second.offset;
      if (!iter->second.instructions.empty() && block_offset >= current_offset &&
          block_offset < cell_end)
      {
        iter = jit_block_db.erase(iter);
      }
      else
      {
        ++iter;
      }
    }
    lock.unlock();
    // we can drop the lock here, because JitTieredCommon::FindBlock acquires its lock on the
    // dispatch cache while holding the lock on the block DB (see also there)
    lock = std::unique_lock(disp_cache_mutex);

    for (auto& disp_entry : disp_cache)
    {
      if ((disp_entry.address & FLAG_MASK) == 1 && disp_entry.offset >= current_offset &&
          disp_entry.offset < cell_end)
      {
        disp_entry.address = 0;
      }
    }
    codespace.SetOffset(current_offset);
  }
  codespace.Emit(compiler.instructions);
  block.offset = current_offset;
  std::unique_lock lock(block_db_mutex);
  jit_block_db.emplace(address, block);
}
