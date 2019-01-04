// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <bitset>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

#include "Core/PowerPC/JitCommon/JitBase.h"
#include "Core/PowerPC/JitCommon/JitCache.h"
#include "Core/PowerPC/PowerPC.h"

/// generic path of the Tiered JIT framework.
/// implements an interpreter with block caching and compaction of said cache.
/// JIT-related data and functions are only allowed here if the data
/// has to be managed by the interpreter. (e. g. dispatch cache, invalidation filter)
class JitTieredGeneric : public JitBase
{
public:
  enum ExecutorFlags : u32
  {
    BLOCK_OVERRUN = 1,
    REPORT_IMMEDIATELY = 2,
    REPORT_BAIL = 4,
  };
  struct Bail
  {
    u32 guest_address;
    u32 status;
  };

  virtual const char* GetName() const { return "TieredGeneric"; }
  virtual void Init() {}
  virtual void ClearCache() final;
  virtual void SingleStep() final;
  virtual void Run();
  virtual void Shutdown() {}

  /// JITs may never reclaim code space while the CPU thread is in JIT code,
  /// so clearing is always safe
  virtual void ClearSafe() final { ClearCache(); }
  virtual bool HandleFault(uintptr_t, SContext*) { return false; }
  virtual void InvalidateICache(u32 address, u32 size, bool forced) final;

  virtual int GetHostCode(u32* address, const u8** code, u32* code_size)
  {
    // generic path has no machine code
    *code_size = 0;
    return 1;
  }
  // these three do nothing for now (but might very well in the future)
  virtual void EnableProfiling(bool enabled) {}
  virtual void GetProfileResults(Profiler::ProfileStats* prof_stats) {}
  virtual void CompileExceptionCheck(JitInterface::ExceptionType) {}

  static bool IsRedispatchInstruction(const UGeckoInstruction inst);

  static constexpr u32 EXCEPTION_SYNC =
      ~(EXCEPTION_EXTERNAL_INT | EXCEPTION_PERFORMANCE_MONITOR | EXCEPTION_DECREMENTER);

protected:
  // for invalidation of JIT blocks
  using Bloom = u64;
  using Executor = u32 (*)(JitTieredGeneric* self, u32 offset, PowerPC::PowerPCState* ppcState,
                           void* toc);
  using InterpreterFunc = void (*)(UGeckoInstruction);

  struct DispatchCacheEntry
  {
    u32 address;
    u32 offset;
    Executor executor;
    Bloom bloom;
    u32 length;
    u32 usecount;

    bool IsValid() const { return executor != nullptr; }
    void Invalidate() { executor = nullptr; }
  };
  static_assert(sizeof(DispatchCacheEntry) <= 32, "Dispatch cache entry should fit in 32 bytes");

  enum InstructionFlags : u16
  {
    // this instruction causes FPU Unavailable if MSR.FP=0 (not checked in the interpreter
    // functions themselves)
    USES_FPU = 1,
    // this instruction has been known to access addresses that are not optimizable RAM addresses
    // currently checked for D-form load/stores on first execution
    NEEDS_SLOWMEM = 2,
  };

  struct DecodedInstruction
  {
    InterpreterFunc func;
    UGeckoInstruction inst;
    /// prefix sum of estimated cycles from start of block
    u16 cycles;
    /// see Instructionflags
    u16 flags;
  };
  static_assert(sizeof(DecodedInstruction) <= 16, "Decoded instruction should fit in 16 bytes");

  struct Invalidation
  {
    u32 first;
    u32 last;
  };
  struct BaselineReport
  {
    std::vector<DispatchCacheEntry> blocks;
    std::vector<DecodedInstruction> instructions;
    std::vector<Invalidation> invalidations;
    std::vector<Bail> bails;
    Bloom invalidation_bloom = BloomNone();
  };

  template <int bits, int shift_off>
  static constexpr u32 XorFold(u32 address)
  {
    address >>= shift_off;
    for (int i = 32 - shift_off; i > 0; i -= bits)
    {
      address = (address >> bits) ^ (address & ((1 << bits) - 1));
    }
    return address;
  }

  static constexpr u32 DispatchCacheKey(u32 addr) { return XorFold<DISP_CACHE_SHIFT, 2>(addr); }

  static Bloom BloomNone() { return 0; }
  static Bloom BloomAll() { return ~BloomNone(); }
  static Bloom BloomCacheline(u32 address) { return u64(1) << XorFold<6, 0>(address >> 5); }
  static Bloom BloomRange(u32 first, u32 last)
  {
    // we only have (guest) cacheline resolution (32=2^5 bytes)
    first >>= 5;
    last >>= 5;
    Bloom res = BloomNone();
    while (first <= last)
    {
      res |= u64(1) << XorFold<6, 0>(first);
      first += 1;
    }
    return res;
  }

  static u32 InterpreterExecutor(JitTieredGeneric* self, u32 offset,
                                 PowerPC::PowerPCState* ppcState, void* toc)
  {
    return InterpretBlock(&self->next_report.instructions[offset]) ? BLOCK_OVERRUN : 0;
  }

  void CompactInterpreterBlocks(BaselineReport* report, bool keep_old_blocks);
  static bool InterpretBlock(const DecodedInstruction* instructions);
  virtual void HandleOverrun(DispatchCacheEntry* entry);
  void RunZeroInstruction();

  DispatchCacheEntry* FindBlock(u32 address);
  /// tail-called by FindBlock if it doesn't find a block in the dispatch cache.
  /// the default implementation just creates a new interpreter block.
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry* entry, u32 address);

  static constexpr int DISP_CACHE_SHIFT = 9;
  static constexpr size_t DISP_PRIMARY_CACHE_SIZE = 1 << DISP_CACHE_SHIFT;
  static constexpr int VICTIM_SETS_SHIFT = 6;
  static constexpr size_t VICTIM_SETS = 1 << VICTIM_SETS_SHIFT;
  static constexpr int VICTIM_WAYS_SHIFT = 5;
  static constexpr size_t VICTIM_WAYS = 1 << VICTIM_WAYS_SHIFT;
  static constexpr size_t DISP_CACHE_SIZE = DISP_PRIMARY_CACHE_SIZE + VICTIM_SETS * VICTIM_WAYS;

  Executor interpreter_executor = InterpreterExecutor;

  /// direct-associative cache (hash table) for blocks, followed by victim cache
  /// (contiguous for iteration convenience)
  /// JIT threads may only delete entries, not create them (safety invariant)
  std::array<DispatchCacheEntry, DISP_CACHE_SIZE> dispatch_cache{};

  /// second-chance bits for the WS-Clock eviction algorithm
  std::bitset<VICTIM_SETS * VICTIM_WAYS> victim_second_chance{};
  /// clocks for the WS-Clock eviction algorithm
  std::array<u8, VICTIM_SETS> victim_clocks{};

  ValidBlockBitSet valid_block;

  BaselineReport next_report;

  /// offset in next_report.instructions at which new instructions begin
  size_t offset_new = 0;
};

// deadlock prevention:
// (A < B means never block on A while holding B)
// report_mutex < block_db_mutex
// report_mutex < disp_cache_mutex
// only the CPU thread may lock block_db_mutex and disp_cache_mutex at the same time
class JitTieredCommon : public JitTieredGeneric
{
public:
  JitTieredCommon();
  virtual void Run() final;

protected:
  struct JitBlock
  {
    Bloom bloom;
    Executor executor;
    u64 runcount;
    u32 offset;
    std::vector<DecodedInstruction> instructions;
    /// this should be sorted.
    std::vector<Bail> bails;
  };
  struct ReportedBlock
  {
    u32 start;
    u32 len;
    u64 usecount;
    std::vector<Bail> bails;
  };

  void CPUDoReport(bool wait, bool hint);
  virtual void HandleOverrun(DispatchCacheEntry*) final;
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry*, u32 address) override;
  static u32 DropLockBeforeInterpreting(JitTieredGeneric* self, u32 offset,
                                        PowerPC::PowerPCState* ppcState, void* toc);

  bool BaselineIteration();
  void UpdateBlockDB(Bloom bloom, std::vector<Invalidation>* invalidations,
                     std::vector<Bail>* bails, std::map<u32, ReportedBlock>* reported_blocks);
  virtual void BaselineCompile(u32 address, JitBlock&& block) = 0;

  static constexpr u32 BASELINE_THRESHOLD = 16;
  static constexpr u32 REPORT_THRESHOLD = 128;
  /// FIXME
  static constexpr size_t CACHELINE = 128;

  // === CPU thread data ===
  /// bloom filter for invalidations that have been reported to Baseline in the last report
  /// (after the next report we can be sure Baseline has processed the invalidations
  /// and don't need to consider this filter anymore)
  Bloom old_bloom = BloomNone();

  void* current_toc = nullptr;

  /// whether to run the Baseline JIT on the CPU thread
  /// (override with false in subclasses, except for debugging)
  bool on_thread_baseline = true;

  u32 (*enter_baseline_block)(JitTieredCommon* self, u32 offset, PowerPC::PowerPCState* ppcState,
                              void* instrumentation_buffer) = nullptr;
  u32 (*enter_optimized_block)(JitTieredCommon* self, u32 offset,
                               PowerPC::PowerPCState* ppcState) = nullptr;

  // === dispatch cache locking ===
  /// holds a lock on disp_cache_mutex most of the time
  std::unique_lock<std::mutex> cpu_thread_lock;
  /// only unlocked by CPU thread when looking up JIT blocks.
  /// only locked by other threads on code space reclamation.
  std::mutex disp_cache_mutex;

  // === Block DB ===
  alignas(CACHELINE) std::mutex block_db_mutex;
  std::map<u32, JitBlock> jit_block_db;

  // === Baseline report ===
  alignas(CACHELINE) std::mutex report_mutex;
  std::condition_variable report_cv;
  bool report_sent = false;
  bool quit = false;
  BaselineReport baseline_report;
  void* next_toc = nullptr;

  // === Baseline thread data ===
  alignas(CACHELINE) std::map<u32, u32> block_counters;
  u32 current_offset = 0;
};
