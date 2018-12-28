// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/TieredPPC64.h"

#include <cinttypes>

#include "Core/HW/CPU.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

JitTieredPPC64::JitTieredPPC64()
{
  codespace.AllocCodeSpace(CODESPACE_CELL_SIZE * CODESPACE_CELLS * 4);
  for (u32 opcode = 0; opcode < 64; ++opcode)
  {
    toc.fallback_table[opcode] = PPCTables::GetInterpreterOp(opcode << 26);
  }
  std::array<u32, 4> arr = {4, 19, 31, 63};
  for (u32 i = 0; i < 4; ++i)
  {
    u32 opcode = arr[i];
    for (u32 subop = 0; subop < 1024; ++subop)
    {
      toc.fallback_table[64 + i * 1024 + subop] =
          PPCTables::GetInterpreterOp((opcode << 26) | (subop << 1));
    }
  }
  for (u32 subop = 0; subop < 32; ++subop)
  {
    toc.fallback_table[64 + 4 * 1024 + subop] =
        PPCTables::GetInterpreterOp((59u << 26) | (subop << 1));
  }
  toc.check_exceptions = PowerPC::CheckExceptions;
  toc.check_external_exceptions = PowerPC::CheckExternalExceptions;
  current_toc = next_toc = &toc;
  on_thread_baseline = false;
}

JitTieredGeneric::DispatchCacheEntry* JitTieredPPC64::LookupBlock(DispatchCacheEntry* entry,
                                                                  u32 address)
{
  JitTieredCommon::LookupBlock(entry, address);
  if (entry->executor != interpreter_executor)
  {
    asm volatile("isync");
  }
  return entry;
}

void JitTieredPPC64::ReclaimCell(u32 cell)
{
  auto cell_start = reinterpret_cast<Executor>(codespace.GetPtrAtIndex(CODESPACE_CELL_SIZE * cell));
  auto cell_end =
      reinterpret_cast<Executor>(codespace.GetPtrAtIndex(CODESPACE_CELL_SIZE * (cell + 1)));
  WARN_LOG(DYNA_REC, "reclaiming Baseline code space cell %u", cell);
  std::unique_lock lock(block_db_mutex);

  for (auto iter = jit_block_db.begin(); iter != jit_block_db.end(); ++iter)
  {
    auto executor = iter->second.executor;
    if (executor >= cell_start && executor < cell_end)
    {
      iter->second.executor = nullptr;
    }
  }
  lock.unlock();
  // we can drop the lock here, because JitTieredCommon::LookupBlock acquires its lock on the
  // dispatch cache while holding the lock on the block DB (see also there)
  lock = std::unique_lock(disp_cache_mutex);

  for (auto& disp_entry : dispatch_cache)
  {
    if (disp_entry.executor < cell_end && disp_entry.executor >= cell_start)
    {
      disp_entry.Invalidate();
    }
  }
}

void JitTieredPPC64::BaselineCompile(u32 address, JitBlock&& block)
{
  PPC64BaselineCompiler compiler;
  std::vector<UGeckoInstruction> instructions;
  for (const DecodedInstruction& inst : block.instructions)
  {
    instructions.push_back(UGeckoInstruction(inst.inst));
  }
  compiler.Compile(address, instructions);
  const u32 len = compiler.instructions.size();
  if (len > CODESPACE_CELL_SIZE)
  {
    WARN_LOG(DYNA_REC, "Huge block (%u instructions) compiled. Refusing to emit.", len);
    return;
  }
  if (len > CODESPACE_CELL_SIZE - offset_in_cell)
  {
    current_cell = (current_cell + 1) % CODESPACE_CELLS;
    ReclaimCell(current_cell);
    codespace.SetOffset(current_cell * CODESPACE_CELL_SIZE);
    offset_in_cell = 0;
  }
  codespace.Emit(compiler.instructions);
  current_offset = CODESPACE_CELL_SIZE * current_cell + offset_in_cell;
  block.offset = current_offset;
  block.executor = reinterpret_cast<Executor>(codespace.GetPtrAtIndex(current_offset));
  offset_in_cell += len;
  INFO_LOG(DYNA_REC, "created executable code at 0x%016" PRIx64 " (offset %u, size %zu)",
           u64(block.executor), current_offset, compiler.instructions.size());
  // CPU::Break();
  std::unique_lock lock(block_db_mutex);
  jit_block_db.emplace(address, block);
}
