#pragma once
//#include <cinttypes>
#include "Common/CommonTypes.h"
#include "Common/Assert.h"
#include "Core/PowerPC/JitCommon/JitBase.h"

struct RegisterState
{
  bool spilled;
};

class JitPpc64Emitter
{
private:
  u32 *code_ptr;
public:
  void SetCodePtr(u8 *ptr)
  {
    code_ptr = (u32*)(((u64)ptr + 3) & ~(u64)3);
  }
  const u8 *GetCodePtr() const
  {
    return (u8 *)code_ptr;
  }
  u8 *GetWritableCodePtr() const
  {
    return (u8 *)code_ptr;
  }
  void WriteInstruction(u32 inst)
  {
    *(code_ptr++) = inst;
  }
  void AlignToCache()
  {
    code_ptr = (u32*)(((u64)code_ptr + 7) & ~(u64)7);
  }
  void SetBranchTarget(u8 *ptr)
  {
    u32 *target = code_ptr;
    SetCodePtr(ptr);
    _assert_msg_(DYNA_REC, (u64)target < (u64)ptr, "backward branch");
    uintptr_t distance = ((u64)target - (u64)ptr);
    if ((*code_ptr >> 26) == 16)
    {
      _assert_msg_(DYNA_REC, distance > 0xfffc, "branch too far");
      *code_ptr |=  (u32)distance & 0xfffc;
    }
    else if ((*code_ptr >> 26) == 18)
    {
      _assert_msg_(DYNA_REC, distance > 0xfffc, "jump too far");
      *code_ptr |=  (u32)distance & 0x3fffffc;
    }
    else
    {
      _assert_msg_(DYNA_REC, false, "not a supported branch instruction");
    }
    code_ptr = target;
  }
};

class Ppc64CodeBlock : public CodeBlock<JitPpc64Emitter>
{
private:
  void PoisonMemory() override
  {
    constexpr u32 trap = 0x7fe00000;
    for (size_t i = 0; i < region_size; i += sizeof(u32))
    {
      std::memcpy(region + i, &trap, sizeof(u32));
    }
  }
};

class JitPpc64BlockCache : public JitBaseBlockCache
{
public:
  static constexpr int CACHE_SETS_SHIFT = 8;
  static constexpr int CACHE_SETS = 1 << CACHE_SETS_SHIFT;
  // Exponent of two for the number of ways per cache set, max. 8
  static constexpr int CACHE_WAYS_SHIFT = 2;
  static constexpr int CACHE_WAYS = 1 << CACHE_WAYS_SHIFT;
  static constexpr int CACHE_SLOTS = 1 << (CACHE_SETS_SHIFT+CACHE_WAYS_SHIFT);
  struct alignas(64) DispatchCache
  {
    u64 values[CACHE_SLOTS];
    u32 tags[CACHE_SLOTS];
    u8 clocks[CACHE_SETS];
  };
  DispatchCache dispatch_cache;
  // search for a matching block in fast cache
  // the invalidate option clears the entry after reading it, and is mostly useful for nested returns
  static void *DispatchFast(DispatchCache *cache, u32 addr, bool invalidate);
  // insert or overwrite a block mapping in fast cache, evicting as necessary
  static void PutCache(DispatchCache *cache, u32 addr, void *target);
private:
  void WriteLinkBlock(const JitBlock::LinkData& source, const JitBlock* dest) {_assert_msg_(DYNA_REC, false, "unimplemented");}
};

class JitPpc64 : public JitBase
{
private:
  struct FastMemInfo {
    u32 entry_point;
    u32 pc;
    RegisterState reg_state;
  };

  std::unordered_map<u64, FastMemInfo> fastmem_info;
  JitPpc64BlockCache block_cache;
  Ppc64CodeBlock code_block;
  CommonAsmRoutinesBase millicode_addresses;

  void GenerateMillicode();
  void Init() override;
  static void *DispatchSlow(JitPpc64 *jit);
public:
  JitPpc64();
  ~JitPpc64() override;

  virtual JitBaseBlockCache* GetBlockCache() override { return &block_cache; }

  virtual void Jit(u32 em_address) override {_assert_msg_(DYNA_REC, false, "unimplemented");}

  virtual const CommonAsmRoutinesBase* GetAsmRoutines() override { return &millicode_addresses; }

  virtual bool HandleFault(uintptr_t access_address, SContext* ctx) override {_assert_msg_(DYNA_REC, false, "unimplemented"); return false;}
  virtual bool HandleStackFault() override { return false; }
};
