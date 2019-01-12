// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/TieredDummy.h"

#include <unordered_set>

#include "Common/Logging/Log.h"
#include "Core/PowerPC/PPCTables.h"

// does basically the same as JitTieredGeneric::InterpreterExecutor
u32 JitTieredDummy::Executor(JitTieredGeneric* self, u32 offset, PowerPC::PowerPCState* ppcState,
                             void*)
{
  JitTieredDummy* xself = static_cast<JitTieredDummy*>(self);
  // INFO_LOG(DYNA_REC, "running @ %08x", block.address);
  const DecodedInstruction* inst = &xself->baseline_codespace[offset];
  return InterpretBlock(inst);
}

void JitTieredDummy::Init()
{
  JitTieredCommon::Init();
  on_thread_baseline = false;
}

void JitTieredDummy::ProvideSpace(u32 len)
{
  u32 current_cell = current_offset / CODESPACE_CELL_SIZE;
  u32 next_cell = ((current_offset + len + 1) / CODESPACE_CELL_SIZE) % CODESPACE_CELLS;
  if (current_cell != next_cell)
  {
    current_offset = next_cell * CODESPACE_CELL_SIZE;
    WARN_LOG(DYNA_REC, "reclaiming Baseline code space cell %u (%x – %x)", next_cell,
             current_offset, current_offset + CODESPACE_CELL_SIZE);

    std::unique_lock lock(block_db_mutex);

    auto iter = jit_block_db.begin();
    for (; iter != jit_block_db.end(); ++iter)
    {
      auto iter2 = iter->second.entry_points.begin();
      while (iter2 != iter->second.entry_points.end())
      {
        u32 block_offset = iter2->offset;
        if (block_offset >= current_offset && block_offset < current_offset + CODESPACE_CELL_SIZE)
        {
          iter2 = iter->second.entry_points.erase(iter2);
        }
        else
        {
          ++iter2;
        }
      }
    }

    lock.unlock();
    // we can drop the lock here, because JitTieredCommon::LookupBlock acquires its lock on the
    // dispatch cache while holding the lock on the block DB (see also there)
    lock = std::unique_lock(disp_cache_mutex);

    for (auto& disp_entry : dispatch_cache)
    {
      if (disp_entry.executor == Executor && disp_entry.offset >= current_offset &&
          disp_entry.offset < current_offset + CODESPACE_CELL_SIZE)
      {
        disp_entry.Invalidate();
      }
    }
    // at this point we can be sure that the CPU thread won't use the invalidated blocks anymore
  }
}

void JitTieredDummy::BaselineCompile(std::vector<u32> suggestions)
{
  for (auto job : PrepareBaselineSuggestions(suggestions))
  {
    u32 len = job.instructions.size();
    INFO_LOG(DYNA_REC, "'compiling' block @ %08x (%u instructions)", job.address, len);
    if (len >= CODESPACE_CELL_SIZE)
    {
      WARN_LOG(DYNA_REC, "Huge block (%u instructions) detected. Refusing to 'compile'.", len);
      return;
    }
    ProvideSpace(len);
    CompiledBlock block{};
    block.executor = Executor;
    block.offset = current_offset;
    for (u32 i = 0; i < len; i += 1)
    {
      baseline_codespace[current_offset + i] = job.instructions[i];
    }
    baseline_codespace[current_offset + len] = {};
    current_offset += len + 1;
    block.guest_length = len;
    block.host_length = sizeof(DecodedInstruction) * len;
    block.bloom = BloomRange(job.address, job.address + len - 1);
    block.additional_bbs = std::move(job.additional_bbs);
    AddJITBlock(job.address, std::move(block));
  }
}
