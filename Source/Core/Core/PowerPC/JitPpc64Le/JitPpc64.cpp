JitPpc64::JitPpc64()
{
  // 32 MiB is the largest distance where we can reliably jump using a single instruction
  // (B has a 24-bit signed immediate distance which is shifted by 2),
  // so let's use that as code space size
  code_block.AllocCodeSpace(0x2000000);
  dispatch_cache_aligned = (uintptr_t *)(((uintptr_t)dispatch_cache + 63) & ~(uintptr_t)63);
  Init();
}

void JitPpc64::Init()
{
  // Millicode routines - remember: codestacks expect the instructions in reverse order!
  Ppc64Instructions::CodeStack cs;
  // downcount
  cs.stw(r4, r3, (s16)offsetof(PowerPC::PowerPCState, downcount));
  cs.mfctr(r4);
  cs.bc(BC_DECREMENT_ONLY | BC_DZ, 0, 0, false);
  cs.mtctr(r4);
  static_assert(sizeof(int) == 4, "int is not word-sized");
  cs.lwz(r4, r3, (s16)offsetof(PowerPC::PowerPCState, downcount));
  cs.GenerateConstant((u64)&PowerPC::ppcState);
  // enter code: save nonvolatile GPRs/FPRs
  s16 gpr_save_size = 18*8;
  s16 fpr_save_size = 18*8;
  s16 stackframe_size = 48+gpr_save_size+fpr_save_size;
  for (int i = 14; i < 32; i += 1)
  {
    cs.std((PpcGpr)i, r1, 48+16*i);
    cs.stfd((PpcFpr)i, 48+gpr_save_size+16*i);
  }
  // save return pointer
  cs.std(r3, r1, 16);
  cs.mflr(r3);
  // save CR
  cs.stw(r3, r1, 8);
  cs.mfcr(r3);
  // bump stack pointer and write back link
  cs.addi(r1, r1, stackframe_size);
  cs.std(r1, r1, -stackframe_size);
  GenerateMilliCode();
}

JitPpc64::~JitPpc64()
{
  code_block.FreeCodeSpace();
}
