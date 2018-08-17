#include "Core/PowerPC/JitPpc64Le/JitPpc64.h"
#include "Core/PowerPC/JitPpc64Le/Ppc64Instructions.h"
#include "Core/PowerPC/PowerPC.h"
#include "Core/CoreTiming.h"

static void bail()
{
  // do things
  CoreTiming::Idle();
  PowerPC::ppcState.downcount = 1024;
}

void *JitPpc64::DispatchSlow(JitPpc64 *jit)
{
  const u8 *ptr = jit->block_cache.Dispatch();
  u32 pc = PC;
  if (!ptr)
  {
    jit->Jit(pc);
    ptr = jit->block_cache.Dispatch();
  }
  _assert_(!!ptr);
  JitPpc64BlockCache::PutCache(&jit->block_cache.dispatch_cache, pc, (void*)ptr);
  return (void*)ptr;
}

constexpr int hash_address(u32 addr, int shift)
{
  int steps = (31 + shift) / shift;
  u32 addr_shifted = addr;
  u32 hash = addr;
  for (steps -= 1;steps > 0; steps -= 1)
  {
    addr_shifted >>= shift;
    hash ^= addr_shifted;
  }
  return hash & ((1 << shift) - 1);
}

void *JitPpc64BlockCache::DispatchFast(DispatchCache *cache, u32 addr, bool invalidate)
{
  int set = hash_address(addr, CACHE_SETS_SHIFT);
  int start = set * CACHE_WAYS;
  for (int way = 0; way < CACHE_WAYS; way += 1)
  {
    if (cache->tags[start + way] == addr)
    {
      u64 res = cache->values[start + way] & ~1;
      // either invalidate the entry or set the R-bit (LSB)
      cache->values[start + way] = invalidate ? 0 : res;
      return (void*)res;
    }
  }
  return nullptr;
}

void JitPpc64BlockCache::PutCache(DispatchCache *cache, u32 addr, void *target)
{
  int set = hash_address(addr, CACHE_SETS_SHIFT);
  int start = set * CACHE_WAYS;
  // look for invalid entries first
  for (int i = 0; i < CACHE_WAYS; i += 1)
  {
    if (cache->values[start + i] == 0)
    {
      cache->values[start + i] = (u64)target;
      cache->tags[start + i] = addr;
      return;
    }
  }
  u8 clock = cache->clocks[set];
  for (int i = 0; i < CACHE_WAYS; i += 1)
  {
    int pos = (clock + i) % CACHE_WAYS + start;
    if ((cache->values[pos] & 1) == 1)
    {
      clock = pos % CACHE_WAYS;
      break;
    }
    cache->values[pos] ^= 1;
  }
  cache->clocks[set] = clock;
  cache->values[start + clock] = (u64)target;
  cache->tags[start + clock] = addr;
}


/*void JitPpc64::GenerateMillicode()
{
  using namespace Ppc64Instructions;
  // constant pool
  u64 commonConstants[] = {
    (u64)&block_cache.dispatch_cache,
    (u64)&PowerPC::ppcState,
    (u64)&JitPpc64BlockCache::DispatchFast,
    (u64)&bail,
    (u64)&DispatchSlow,
    (u64)this
  };
  Ppc64CodeBlock& cb = code_block;
  cb.AlignToCache();
  u64 constantpool = (u64) code_block.GetCodePtr();
  for (int i = 0; i < (sizeof(commonConstants) / 8); i += 1)
  {
    cb.WriteInstruction((u32)(commonConstants[i] & 0xffffffff));
    cb.WriteInstruction((u32)(commonConstants[i] >> 32));
  }
  cb.AlignToCache();
  millicode_addresses.enterCode = cb.GetCodePtr();
  s16 gpr_save_size = 18*8;
  s16 fpr_save_size = 18*8;
  s16 stackframe_size = 48+gpr_save_size+fpr_save_size;
  cb.WriteInstruction(std_(r1, r1, -stackframe_size));  // write back chain pointer
  cb.WriteInstruction(addi(r1, r1, stackframe_size));  // bump stack pointer
  // dump nonvolatile registers
  cb.WriteInstruction(mfcr(r3));
  cb.WriteInstruction(stw(r3, r1, 8));
  cb.WriteInstruction(mfspr(r3, 8));  // mflr
  cb.WriteInstruction(std_(r3, r1, 16));
  for (int i = 14; i < 32; i += 1)
  {
    cb.WriteInstruction(std_((PpcGpr)i, r1, 48+16*i));
    cb.WriteInstruction(stfd((PpcGpr)i, r1, 48+gpr_save_size+8*i));
  }
  // load the dispatcher pointer (doubles as constant pool pointer) into r2
  cb.WriteInstruction(addpcis(r2, 0));
  // dispatcher: start by decreasing downcount
  millicode_addresses.dispatcher = cb.GetCodePtr();
  cb.WriteInstruction(mfspr(r31, 8));  // dispatched entry points take LR in r31 to compensate for the impossibility of virtual tail calls
  s16 constpool_offset = -(s16)((u64)millicode_addresses.dispatcher - constantpool);
  cb.WriteInstruction(ld(r3, r2, constpool_offset + 8));
  static_assert(sizeof(int) == 4, "int is not word-sized");
  cb.WriteInstruction(lwz(r4, r3, (s16)offsetof(PowerPC::PowerPCState, downcount)));
  u8 *branch_bail = cb.GetWritableCodePtr();
  cb.WriteInstruction(bc(BC_DECREMENT_ONLY | BC_DZ, 0, 0, false));
  cb.WriteInstruction(mfspr(r4, 9)); // mfctr
  cb.WriteInstruction(stw(r4, r3, (s16)offsetof(PowerPC::PowerPCState, downcount)));

  millicode_addresses.dispatcherNoCheck = cb.GetCodePtr();
  cb.WriteInstruction(ld(r5, r2, constpool_offset + 8));
  cb.WriteInstruction(ld(r4, r5, (s16)offsetof(PowerPC::PowerPCState, pc)));
  cb.WriteInstruction(ld(r3, r2, constpool_offset));
  cb.WriteInstruction(or_(r30, r2, r2)); // save dispatcher pointer (r2 is volatile across cross-module calls)
  cb.WriteInstruction(mtspr(r15, 8)); // mtlr
  cb.WriteInstruction(bclr(BC_ALWAYS, 0, Predictable, true));
  cb.WriteInstruction(or_(r2, r30, r30)); // restore dispatcher pointer
  cb.WriteInstruction(cmpdi(crf0, r3, 0));
  u8 *branch_notfound = cb.GetWritableCodePtr();
  cb.WriteInstruction(bc(0, 2, 0, false));
  // calculate cache set
  cb.WriteInstruction(or(r16, r15, r15));
  cb.WriteInstruction(rlwinm(r17, r15, 32-CACHE_SETS_SHIFT, CACHE_SETS_SHIFT, 31)); // shift right
  cb.WriteInstruction(xor(r16, r16, r17));
  int steps = (31 + CACHE_SETS_SHIFT) / CACHE_SETS_SHIFT;
  for (int i = 1; i < steps; i += 1)
  {
    cb.WriteInstruction(rlwinm(r17, r15, 32-CACHE_SETS_SHIFT, CACHE_SETS_SHIFT, 31)); // shift right
    cb.WriteInstruction(xor(r16, r16, r17));
  }
  cb.WriteInstruction(rlwinm(r16, r16, CACHE_WAYS_SHIFT + 2, 29-CACHE_SETS_SHIFT-CACHE_WAYS_SHIFT, 29-CACHE_SETS_SHIFT)); // CACHE_SETS_SHIFT bits shifted left by CACHE_WAYS_SHIFT + 2 bits
  // we now have the set id * #ways * 4 in r16
  u8 *branch_notfound;
  if (DISPATCH_UNROLL)
  {
    cb.WriteInstruction(addi(r19, r18, 8*CACHE_SLOTS + 8));
    cb.WriteInstruction(add(r19, r16, r19));
    if (CACHE_WAYS_SHIFT > 3)
    {
      abort();
    }
    // read all ways in a separate register and test them in a separate CR field
    for (int i = 0; i < CACHE_WAYS; i += 1)
    {
      cb.WriteInstruction(lwz(r20+i, r19, (i16)i*4));
      cb.WriteInstruction(cmpw(i, r20, r15));
    }
    // then test all the zero bits and go to the corresponding block from the next loop
    // jump distance calculation is pretty subtle here
    for (int i = 0; i < CACHE_WAYS; i += 1)
    {
      cb.WriteInstruction(bc(0, 2+i*4, 4*(CACHE_WAYS + 1 + i)));
    }
    branch_notfound = cb.GetCodePtr();
    cb.WriteInstruction(b(0));
    for (int i = 0; i < CACHE_WAYS - 1; i += 1)
    {
      // write way number
      cb.WriteInstruction(addi(r20, 0, i));
      // skip past others
      cb.WriteInstruction(b(8*(CACHE_WAYS - 1 - i) + 8);
    }
    cb.WriteInstruction(addi(r20, 0, CACHE_WAYS - 1);
  }
  else
  {
    // start at the last slot in a way
    cb.WriteInstruction(addi(r19, r18, 8*CACHE_SLOTS + 8 + 4*CACHE_WAYS_SHIFT));
    cb.WriteInstruction(add(r19, r16, r19));
    // init counter
    cb.WriteInstruction(addi(r20, 0, CACHE_WAYS));
    cb.WriteInstruction(mtspr(r20, 9)); // mtctr
    // loop begin
    cb.WriteInstruction(lwz(r20, r19, 0));
    cb.WriteInstruction(cmpwi(crf0, r20, 0));
    // break out
    u8 *branch_found = bc.GetCodePtr();
    cb.WriteInstruction(bc(0, 2, 0));
    cb.WriteInstruction(addi(r19, r19, -4));
    cb.WriteInstruction(bc(BC_DECREMENT_ONLY, 0, 0));
    // loop end - not found
    branch_notfound = bc.GetCodePtr();
    cb.WriteInstruction(b(0));
    cb.SetBranchTarget(branch_found);
    cb.WriteInstruction(mfspr(r20, 9)); // mfctr
    cb.WriteInstruction(addi(r20, r20, -1);
  }
  // regardless of how we searched for it, we now have a way number in r20
  // multiply by 8 to get offset within set
  cb.WriteInstruction(rlwinm(r20, r20, 3, 0, 29));  // shift way id left by 3
  // scale by two, because values are 64bit, while tags are 32bit
  cb.WriteInstruction(rlwinm(r16, r16, 1, 0, 30));  // shift set offset left by 1
  // sum both offsets to the table base
  cb.WriteInstruction(add(r19, r18, r20));
  cb.WriteInstruction(add(r19, r19, r16));
  // load jump target
  cb.WriteInstruction(ld(r21, r19, 0)
  cb.WriteInstruction(mtspr(r3, 8));  // mtlr
  // reload volatile registers if necessary
  cb.WriteInstruction(bclr(BC_ALWAYS, 0, Unpredictable, false));

  // if not found, compile
  cb.SetBranchTarget(branch_notfound);
  cb.WriteInstruction(ld(r3, r2, constantpool + 32));
  cb.WriteInstruction(or_(r30, r2, r2)); // save dispatcher pointer (r2 is volatile across cross-module calls)
  cb.WriteInstruction(mtspr(r15, 8)); // mtlr
  cb.WriteInstruction(bclr(BC_ALWAYS, 0, Predictable, true));
  cb.WriteInstruction(or_(r2, r30, r30)); // restore dispatcher pointer
  // retry
  cb.WriteInstruction(b(-(s32)((u64)cb.GetCodePtr() - (u64)millicode_addresses.dispatcherNoCheck), false));

  cb.SetBranchTarget(branch_bail);
  cb.WriteInstruction(ld(r15, r2, constantpool + 16));
  cb.WriteInstruction(mtspr(r15, 8)); // mtlr
  cb.WriteInstruction(or_(r30, r2, r2)); // save dispatcher pointer (r2 is volatile across cross-module calls)
  cb.WriteInstruction(bclr(BC_ALWAYS, 0, Predictable, true));
  cb.WriteInstruction(or_(r2, r30, r30)); // restore dispatcher pointer
  // resume
  cb.WriteInstruction(b(-(s32)((u64)cb.GetCodePtr() - (u64)millicode_addresses.dispatcherNoCheck), false));
}*/
