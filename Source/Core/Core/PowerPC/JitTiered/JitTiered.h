#pragma once

#include <atomic>
#include <mutex>

#include "Core/PowerPC/JitCommon/JitBase.h"

// depends on microarchitecture; usual values are 32, 64 or 128; this is used to avoid false sharing, so use the highest value in use on the given architecture
constexpr int CACHELINE_SIZE = 128;

template<typename Inner> struct HandShake {
private:
  /// if we could detect whether a mutex is being waited on (theoretically possible in most mutex implementations) we wouldn't need this
  std::atomic<int> select;
  struct
  {
    // separate into different cache lines to avoid false sharing
    alignas(CACHELINE_SIZE) std::mutex mutex;
    Inner inner;
  } sides[2];
  alignas(CACHELINE_SIZE) int side;
  std::unique_lock<std::mutex> writerGuard;
public:
  class ReaderGuard
  {
  private:
    HandShake &parent;
    std::unique_lock<std::mutex> guard;
  public:
    // announces a switch
    ~ReaderGuard();
    Inner& GetRef();
  };
  // dropping a value of this type will release the other side to the reader
  class YieldGuard
  {
    friend class HandShake<Inner>;
  private:
    HandShake &parent;
    std::unique_lock<std::mutex> guard;
  public:
    /// returns the side that will be freed after dropping this guard
    Inner& GetRef() { return parent.sides[1^parent.side].inner; }
  };
  HandShake();
  
  // reader interface
  std::optional<ReaderGuard> TryRead();
  ReaderGuard Wait();
  
  // writer interface
  Inner &GetWriter() { return sides[side].inner; }
  std::optional<YieldGuard> Yield() {
    if (select.load() != side)
    { // reader is still on the other side
      return {};
    }
    side ^= 1;
    YieldGuard guard = {*this, std::unique_lock(sides[side])};
    writerGuard.swap(guard.guard);
  }
};

class JitTiered : public JitBase
{
private:
  template<int bits>
  static u32 XorFold(u32 address)
  {
    for (int i = 32; i > 0; i -= bits)
    {
      address = (address >> bits) ^ (address & ((1 << bits) - 1));
    }
    return address;
  }
  typedef u64 Bloom;
  typedef void (*InterpreterFunc)(UGeckoInstruction);

  static Bloom BloomNone() { return 0; }
  static Bloom BloomAll() { return ~BloomNone(); }
  static Bloom BloomRange(u32 first, u32 last)
  { // we only have (guest) cacheline resolution;
    first >>= 5;
    last >>= 5;
    Bloom res;
    while (first <= last)
    {
      res |= 1 << XorFold<6>(first);
      first += 1;
    }
    return res;
  }

  struct DecodedInstruction
  {
    UGeckoInstruction inst;
    /// prefix sum of estimated cycles
    u32 cycles;
    InterpreterFunc func;
  };
  // the least-significant two bits of instruction addresses are always zero, so we can use them for flags.
  // this flag is used in all the set-associative caches to implement WS Clock eviction.
  static constexpr int SECOND_CHANCE = 1;
  
  /// try submitting a report to Baseline approximately every 64K downcount
  static constexpr int BASELINE_REPORT_SHIFT = 16;
  
  // === Exec thread data ===
  static constexpr int INT_CACHE_WAYS_SHIFT = 2;
  static constexpr int INT_CACHE_WAYS = 1 << INT_CACHE_WAYS_SHIFT;
  static constexpr int INT_CACHE_SETS_SHIFT = 4;
  static constexpr int INT_CACHE_SETS = 1 << INT_CACHE_SETS_SHIFT;
  static constexpr int INT_CACHE_SIZE = 1 << (INT_CACHE_SETS_SHIFT+INT_CACHE_WAYS_SHIFT);
  /// set-associative cache for code blocks discovered this cycle
  u32 new_blocks_addrs[INT_CACHE_SIZE] {};
  std::vector<DecodedInstruction> new_blocks_instructions[INT_CACHE_SIZE];
  u8 new_blocks_clocks[INT_CACHE_SETS] {};

  /// bloom filter for blocks invalidated this cycle. since interpreter blocks are invalidated immediately,
  /// this is only used when consulting the block table (managed by Baseline ⇒ asynchronously invalidated)
  Bloom invalidation_mask = BloomNone();
  /// when consulting the block table, we need to consider that Baseline may not have removed the invalidated blocks of the last cycle yet, either
  Bloom old_invalidation_mask = BloomNone();

  static constexpr int DISP_CACHE_WAYS_SHIFT = 2;
  static constexpr int DISP_CACHE_SETS_SHIFT = 8;
  static constexpr int DISP_CACHE_SIZE = 1 << (DISP_CACHE_SETS_SHIFT+DISP_CACHE_WAYS_SHIFT);
  /// only released by Exec when consulting the block table. only needs to be accessed by the JITs when compacting their code space.
  std::mutex dispatch_cache_mutex;
  /// set-associative cache for JIT blocks
  u32 dispatch_addrs[DISP_CACHE_SIZE] {};
  /// flag for the dispatch cache indicating that it was made by the 
  static constexpr int OPTIMIZER_BLOCK = 2;
  /// other half of the dispatch cache. separated for density reasons
  struct JitBlockBody
  {
    /// entry point for this block. this is only ever passed to JIT entry code, so no specific pointer type
    void *code;
    /// allow for immediate, probabilistic invalidation on the Exec thread
    Bloom bloom;
  } dispatch_blocks[DISP_CACHE_SIZE];
  u8 dispatch_clocks[1 << DISP_CACHE_SETS_SHIFT] {};
  
  // Exec → Baseline reports
  struct MemoryRange
  {
    u32 start;
    u32 length;
  };
  struct BaselineReport
  {
    /// set-associative cache (immutable save for invalidation) for blocks discovered last cycle, retained until Baseline has ACKed them by initiating the next cycle
    u32 block_addrs[INT_CACHE_SIZE] {};
    u32 block_ends[INT_CACHE_SIZE] {};
    /// copy of compacted_blocks after compaction
    std::vector<DecodedInstruction> instructions;
    /// memory ranges invalidated this cycle
    std::vector<MemoryRange> invalidations;
    /// the contents of this are managed by Baseline.
    /// Generally, the first elements of this are counters for various events
    /// and JIT code may append additional elements that make scanning those counters faster
    std::vector<u32> usage_info;
  };
  HandShake<BaselineReport> baseline_report;
  
  /// Exec → Optimizer reports: usage info as for Baseline
  HandShake<std::vector<u32>> optimizer_report;
  
  // === Baseline thread data ===
  std::mutex block_table_mutex;
  struct JitBlock
  {
    /// zero for optimized blocks
    u32 length;
    JitBlockBody body;
    /// only used for optimized blocks: start address → length
    /// only used by the JIT threads
    std::set<u32, u32> address_ranges;
  };
  std::unordered_map<u32, JitBlock> block_table;
  
  static std::optional<DecodedInstruction> FetchInstruction();
  /// XOR-fold address for interpreter block cache
  static u32 InterpreterCacheKey(u32 address)
  {
    return XorFold<INT_CACHE_SETS_SHIFT>(address >> 2);
  }
  /// XOR-fold address for dispatch cache
  static u32 DispatchCacheKey(u32 address)
  {
    return XorFold<DISP_CACHE_SETS_SHIFT>(address >> 2);
  }
  static std::optional<int> FindInterpreterBlock(u32 *table, u32 key, u32 address);
  std::vector<DecodedInstruction> &CreateFreeBlock(u32 key, u32 address);
  
  void InterpretBlock();
  void CompactInterpreterBlocks();
public:
  virtual const char *GetName() const {return "TieredGeneric";}
  virtual void Init() {}
  virtual void ClearCache();
  virtual void SingleStep();
  virtual void Run();
  virtual void Shutdown();

  virtual void ClearSafe() { ClearCache(); }
  virtual bool HandleFault(uintptr_t, SContext*) { return false; }
  virtual void InvalidateICache(u32 address, u32 size, bool forced);
  void CompileExceptionCheck(JitInterface::ExceptionType type) {}
};
