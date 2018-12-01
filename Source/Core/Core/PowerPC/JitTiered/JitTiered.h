#pragma once

#include <vector>

#include "Common/HandShake.h"
#include "Core/PowerPC/JitCommon/JitBase.h"

class JitTieredGeneric : public JitBase
{
public:
  virtual const char* GetName() const { return "TieredGeneric"; }
  virtual void Init() {}
  virtual void ClearCache();
  virtual void SingleStep();
  virtual void Run();
  virtual void Shutdown() {}

  virtual void ClearSafe() { ClearCache(); }
  virtual bool HandleFault(uintptr_t, SContext*) { return false; }
  virtual void InvalidateICache(u32 address, u32 size, bool forced);

private:
  using Bloom = u64;
  static Bloom BloomNone() { return 0; }
  static Bloom BloomAll() { return ~BloomNone(); }
  static Bloom BloomRange(u32 first, u32 last)
  {  // we only have (guest) cacheline resolution;
    first >>= 5;
    last >>= 5;
    Bloom res = BloomNone();
    while (first <= last)
    {
      res |= (u64)1 << XorFold<6, 0>(first);
      first += 1;
    }
    return res;
  }
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
  /// the least-significant two bits of instruction addresses are always zero, so we can use them
  /// for flags. this constant can be used to mask out the flags
  static constexpr u32 FLAG_MASK = ~(u32)3;
  struct DispCacheEntry
  {
    /// PPC instructions are 4-byte aligned, leaving us 2 flag bits with these values:
    /// 0: interpreter (or invalid, if all 0)
    /// 1: Baseline JIT
    /// 2: Optimized JIT
    u32 address;
    /// if interpreter: offset into inst_cache vector
    /// if JIT: offset to JIT code (passed verbatim to entry function)
    u32 offset;
    union
    {
      /// if JIT block
      Bloom bloom;
      /// if interpreter block
      struct
      {
        u32 len;
        u32 usecount;
      };
    };
  };
  static constexpr int DISP_CACHE_SHIFT = 10;
  static constexpr size_t DISP_CACHE_SIZE = 1 << DISP_CACHE_SHIFT;
  DispCacheEntry disp_cache[DISP_CACHE_SIZE];
  static constexpr u32 DispatchCacheKey(u32 addr) { return XorFold<DISP_CACHE_SHIFT, 2>(addr); }

  typedef void (*InterpreterFunc)(UGeckoInstruction);
  struct DecodedInstruction
  {
    InterpreterFunc func;
    UGeckoInstruction inst;
    /// prefix sum of estimated cycles
    u32 cycles;
  };
  std::vector<DecodedInstruction> inst_cache;
  /// offset in inst_cache at which new instructions
  size_t offset_new;

  /// approximate number of cycles between which to check if Baseline wants new data
  /// (only checked when downcount reaches 0, which is in turn only checked at the end of blocks)
  static constexpr int JIT_THROTTLE = 10000;

  struct BaselineReport
  {
    struct CompactedBlock
    {
      u32 address;
      u32 start;
      u32 usecount;
    };
    /// has sentinel value with address = 0 and start = instructions.size() to make block size
    /// calculation easier
    std::vector<CompactedBlock> blocks;
    std::vector<DecodedInstruction> instructions;
  };
  HandShake<BaselineReport> baseline_report;
};
