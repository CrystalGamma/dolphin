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
  for (; inst->func; ++inst)
  {
    ppcState->npc = ppcState->pc + 4;
    if ((inst->flags & USES_FPU) && !ppcState->msr.FP)
    {
      ppcState->Exceptions |= EXCEPTION_FPU_UNAVAILABLE;
    }
    else
    {
      inst->func(inst->inst);
    }
    if (ppcState->Exceptions & EXCEPTION_SYNC)
    {
      PowerPC::CheckExceptions();
      ppcState->downcount -= inst->cycles;
      ppcState->Exceptions &= ~EXCEPTION_SYNC;
      return 0;
    }
    ppcState->pc += 4;
    if (ppcState->npc != ppcState->pc)
    {
      ppcState->pc = ppcState->npc;
      ppcState->downcount -= inst->cycles;
      return 0;
    }
  }
  ppcState->downcount -= (inst - 1)->cycles;
  return BLOCK_OVERRUN;
}

void JitTieredDummy::Init()
{
  JitTieredCommon::Init();
  on_thread_baseline = false;
}

void JitTieredDummy::BaselineCompile(u32 address, JitBlock&& block)
{
  INFO_LOG(DYNA_REC, "'compiling' block @ %08x", address);
  u32 len = block.instructions.size();
  if (len >= CODESPACE_CELL_SIZE)
  {
    WARN_LOG(DYNA_REC, "Huge block (%u instructions) detected. Refusing to 'compile'.", len);
    return;
  }
  u32 current_cell = current_offset / CODESPACE_CELL_SIZE;
  u32 next_cell = ((current_offset + len + 1) / CODESPACE_CELL_SIZE) % CODESPACE_CELLS;
  if (current_cell != next_cell)
  {
    current_offset = next_cell * CODESPACE_CELL_SIZE;
    WARN_LOG(DYNA_REC, "reclaiming Baseline code space cell %u (%x – %x)", next_cell,
             current_offset, current_offset + CODESPACE_CELL_SIZE);
    std::unique_lock lock(block_db_mutex);
    auto iter = jit_block_db.cbegin();
    while (iter != jit_block_db.cend())
    {
      u32 block_offset = iter->second.offset;
      if (!iter->second.instructions.empty() && block_offset >= current_offset &&
          block_offset < current_offset + CODESPACE_CELL_SIZE)
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

    for (auto& disp_entry : dispatch_cache)
    {
      if (disp_entry.executor == Executor && disp_entry.offset >= current_offset &&
          disp_entry.offset < current_offset + CODESPACE_CELL_SIZE)
      {
        disp_entry.Invalidate();
      }
    }
  }
  for (u32 i = 0; i < len; i += 1)
  {
    baseline_codespace[current_offset + i] = block.instructions[i];
  }
  baseline_codespace[current_offset + len] = {};
  std::unique_lock lock(block_db_mutex);
  block.offset = current_offset;
  block.executor = Executor;
  jit_block_db.emplace(address, block);
  current_offset += len + 1;
}
