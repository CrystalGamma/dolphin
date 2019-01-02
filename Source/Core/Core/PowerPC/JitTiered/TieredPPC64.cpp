// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/PowerPC/JitTiered/TieredPPC64.h"

#include <cinttypes>

#include "Core/CoreTiming.h"
#include "Core/HW/CPU.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/JitTiered/PPC64Baseline.h"

JitTieredPPC64::JitTieredPPC64()
{
  PPC64BaselineCompiler compiler;
  compiler.EmitCommonRoutines();
  routine_offsets = compiler.offsets;
  codespace.AllocCodeSpace(CODESPACE_CELL_SIZE * CODESPACE_CELLS + routine_offsets.end);
  // <= instead of < because we want a copy of the routines at the end
  for (u32 i = 0; i <= CODESPACE_CELLS / ROUTINES_INTERVAL; ++i)
  {
    u32 index = i << 24;
    INFO_LOG(DYNA_REC, "Installing common routines at offset %x (0x%016" PRIx64 ")", index * 4,
             u64(codespace.GetPtrAtIndex(index)));
    codespace.SetOffset(index);
    codespace.Emit(compiler.instructions);
  }
  offset_in_cell = routine_offsets.end;
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
  toc.idle = CoreTiming::Idle;
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
  u32 cell_offset = CODESPACE_CELL_SIZE * cell;
  if (cell % ROUTINES_INTERVAL == 0)
  {
    cell_offset += routine_offsets.end;
  }
  auto cell_start = reinterpret_cast<Executor>(codespace.GetPtrAtIndex(cell_offset / 4));
  const u32 end_offset = CODESPACE_CELL_SIZE * (cell + 1);
  auto cell_end = reinterpret_cast<Executor>(codespace.GetPtrAtIndex(end_offset / 4));
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
  PPC64BaselineCompiler compiler(routine_offsets);
  std::vector<UGeckoInstruction> instructions;
  for (const DecodedInstruction& inst : block.instructions)
  {
    instructions.push_back(UGeckoInstruction(inst.inst));
  }
  compiler.Compile(address, instructions);
  const u32 len = compiler.instructions.size() * 4;
  if (len > MAX_BLOCK_SIZE)
  {
    WARN_LOG(DYNA_REC, "Huge block (%u instructions) compiled. Refusing to emit.", len);
    return;
  }
  if (len > CODESPACE_CELL_SIZE - offset_in_cell)
  {
    current_cell = (current_cell + 1) % CODESPACE_CELLS;
    ReclaimCell(current_cell);
    offset_in_cell = current_cell % ROUTINES_INTERVAL == 0 ? routine_offsets.end : 0;
  }
  current_offset = CODESPACE_CELL_SIZE * current_cell + offset_in_cell;
  codespace.SetOffset(current_offset / 4);
  compiler.RelocateAll(current_offset);
  codespace.Emit(compiler.instructions);
  block.offset = current_offset;
  block.executor = reinterpret_cast<Executor>(codespace.GetPtrAtIndex(current_offset / 4));
  offset_in_cell += len;
  INFO_LOG(DYNA_REC, "created executable code at 0x%016" PRIx64 " (offset %u, size %zu)",
           u64(block.executor), current_offset, compiler.instructions.size());
  std::unique_lock lock(block_db_mutex);
  auto find_result = jit_block_db.find(address);
  if (find_result != jit_block_db.end())
  {
    jit_block_db.erase(find_result);
  }
  jit_block_db.emplace(address, block);
}
