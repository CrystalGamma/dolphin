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
  // for invalidation of JIT blocks
  using Bloom = u64;

  using Executor = u32 (*)(JitTieredGeneric* self, u32 offset, PowerPC::PowerPCState* ppcState,
                           void* toc);
  enum ExecutorFlags : u32
  {
    BLOCK_OVERRUN = 1,
    JUMP_OUT = 2,
    REPORT_BAIL = 4,
    REPORT_IMMEDIATELY = 8,
  };

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

  struct Bail
  {
    u32 guest_address;
    u32 status;

    bool operator<(const Bail& other) const
    {
      return std::tie(guest_address, status) < std::tie(other.guest_address, other.status);
    }
  };
  struct DecodedInstruction
  {
    void (*func)(UGeckoInstruction);
    UGeckoInstruction inst;
    /// prefix sum of estimated cycles from start of block
    u16 cycles;
    /// see Instructionflags
    u16 flags;
  };
  static_assert(sizeof(DecodedInstruction) <= 16, "Decoded instruction should fit in 16 bytes");
  enum InstructionFlags : u16
  {
    // this instruction causes FPU Unavailable if MSR.FP=0 (not checked in the interpreter
    // functions themselves)
    USES_FPU = 1,
    // this instruction has been known to access addresses that are not optimizable RAM addresses
    // currently checked for D-form load/stores on first execution
    NEEDS_SLOWMEM = 2,
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

  static constexpr u32 DispatchCacheKey(u32 addr) { return XorFold<DISP_CACHE_SHIFT, 2>(addr); }

  static constexpr u32 EXCEPTION_SYNC =
      ~(EXCEPTION_EXTERNAL_INT | EXCEPTION_PERFORMANCE_MONITOR | EXCEPTION_DECREMENTER);

  static constexpr int DISP_CACHE_SHIFT = 9;

protected:
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
    return InterpretBlock(&self->next_report.instructions[offset]);
  }

  void CompactInterpreterBlocks(BaselineReport* report, bool keep_old_blocks);
  static u32 InterpretBlock(const DecodedInstruction* instructions);
  bool HandleOverrun(DispatchCacheEntry* entry);
  void RunZeroInstruction();

  DispatchCacheEntry* FindBlock(u32 address);
  /// tail-called by FindBlock if it doesn't find a block in the dispatch cache.
  /// the default implementation just creates a new interpreter block.
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry* entry, u32 address);

  static constexpr size_t MAX_BLOCK_LENGTH = 1024;

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
  struct BaselineCompileJob
  {
    u32 address;
    std::vector<DecodedInstruction> instructions;
    std::vector<u32> additional_bbs;
  };
  JitTieredCommon() { interpreter_executor = CheckBPAndInterpret; }
  virtual void Run() final;

protected:
  struct CompiledBlock
  {
    Executor executor;
    Bloom bloom;
    u32 guest_length;
    u32 offset;
    /// pair contains (fault address, handler address)
    std::vector<std::pair<uintptr_t, uintptr_t>> fault_handlers;
    // this exists mainly so the CPU thread can check if this JIT block contains breakpoints.
    // note that it need not be updated in case of a block split since setting a breakpoint would
    // invalidate the basic block anyway, removing this JIT block via inlined_in and if the
    // breakpoint existed before, it will not let a contiguous block be created in the first place
    std::vector<u32> additional_bbs;
    u32 host_length;
  };
  struct BasicBlock
  {
    std::vector<DecodedInstruction> instructions;
    u64 runcount;
    std::vector<CompiledBlock> entry_points;
    /// list of basic blocks with JIT entry points that inline this code
    std::vector<u32> inlined_in;
  };

  static u32 CheckBPAndInterpret(JitTieredGeneric* self, u32 offset,
                                 PowerPC::PowerPCState* ppcState, void* toc);

  void CPUDoReport(bool wait, bool hint);
  u32 HandleBail(u32 address);
  virtual DispatchCacheEntry* LookupBlock(DispatchCacheEntry*, u32 address) override;

  bool BaselineIteration();
  virtual void BaselineCompile(std::vector<u32> suggestions) = 0;
  std::vector<BaselineCompileJob> PrepareBaselineSuggestions(std::vector<u32> suggestions);
  void AddJITBlock(u32 address, CompiledBlock block);
  std::vector<u32> AllAffectedBlocks(u32 address);

  static constexpr u32 REPORT_THRESHOLD = 128;
  static constexpr u32 INTERPRETER_SCORE = 1;
  static constexpr u32 ADVANCE_SCORE = 1;
  static constexpr u32 BASELINE_THRESHOLD = 16;
  /// FIXME
  static constexpr size_t CACHELINE = 128;

  // === CPU thread data ===
  /// bloom filter for invalidations that have been reported to Baseline in the last report
  /// (after the next report we can be sure Baseline has processed the invalidations
  /// and don't need to consider this filter anymore)
  Bloom old_bloom = BloomNone();

  void* current_toc = nullptr;

  bool breakpoint_handled = false;
  u32 report_score = 0;

  std::map<uintptr_t, uintptr_t> fault_handlers;

  /// whether to run the Baseline JIT on the CPU thread
  /// (override with false in subclasses, except for debugging)
  bool on_thread_baseline = true;

  // === dispatch cache locking ===
  /// holds a lock on disp_cache_mutex most of the time
  std::unique_lock<std::mutex> cpu_thread_lock;
  /// only unlocked by CPU thread when looking up JIT blocks.
  /// only locked by other threads on code space reclamation.
  std::mutex disp_cache_mutex;

  // === Block DB ===
  alignas(CACHELINE) std::mutex block_db_mutex;
  std::map<u32, BasicBlock> jit_block_db;

  // === Baseline report ===
  alignas(CACHELINE) std::mutex report_mutex;
  std::condition_variable report_cv;
  bool report_sent = false;
  bool quit = false;
  BaselineReport baseline_report;
  void* next_toc = nullptr;

  // === Baseline thread data ===
  std::set<Bail> all_bails;
  u32 current_offset = 0;
};
