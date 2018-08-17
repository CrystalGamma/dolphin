#pragma once

namespace Ppc64Instructions {

enum PpcGpr {
  r0 = 0,
  r1 = 1,
  r2 = 2,
  r3 = 3,
  r4 = 4,
  r5 = 5,
  r6 = 6,
  r7 = 7,
  r8 = 8,
  r9 = 9,
  r10 = 10,
  r11 = 11,
  r12 = 12,
  r13 = 13,
  r14 = 14,
  r15 = 15,
  r16 = 16,
  r17 = 17,
  r18 = 18,
  r19 = 19,
  r20 = 20,
  r21 = 21,
  r22 = 22,
  r23 = 23,
  r24 = 24,
  r25 = 25,
  r26 = 26,
  r27 = 27,
  r28 = 28,
  r29 = 29,
  r30 = 30,
  r31 = 31
};
enum PpcFpr {
  f0 = 0,
  f1 = 1,
  f2 = 2,
  f3 = 3,
  f4 = 4,
  f5 = 5,
  f6 = 6,
  f7 = 7,
  f8 = 8,
  f9 = 9,
  f10 = 10,
  f11 = 11,
  f12 = 12,
  f13 = 13,
  f14 = 14,
  f15 = 15,
  f16 = 16,
  f17 = 17,
  f18 = 18,
  f19 = 19,
  f20 = 20,
  f21 = 21,
  f22 = 22,
  f23 = 23,
  f24 = 24,
  f25 = 25,
  f26 = 26,
  f27 = 27,
  f28 = 28,
  f29 = 29,
  f30 = 30,
  f31 = 31
};

enum BranchHint
{
  Return = 0,
  Predictable = 1,
  Unpredictable = 3
};

enum CrField
{
  crf0 = 0,
  crf1 = 1,
  crf2 = 2,
  crf3 = 3,
  crf4 = 4,
  crf5 = 5,
  crf6 = 6,
  crf7 = 7
};

constexpr u8 BC_DECREMENT_ONLY = 16;
constexpr u8 BC_DZ = 6;
constexpr u8 BC_DNZ = 4;
constexpr u8 BC_UNSET = 8;
constexpr u8 BC_ALWAYS = 16;

constexpr u32 b(s32 distance, bool link)
{
  return (16 << 26) | ((u32)distance & 0x3fffffc) | (u32) link;
}
constexpr u32 bc(u8 flags, u8 cr_bit, s16 distance, bool link)
{
  return (16 << 26) | (((u32)(flags ^ BC_DNZ ^ BC_UNSET) << 21) & 0x3e0000) | (((u32)cr_bit << 16) & 0x1f0000) | ((u32)distance & 0xfffc) | (u32) link;
}

constexpr u32 bclr(u8 flags, u8 cr_bit, BranchHint hint, bool link)
{
  return (16 << 26) | (((u32)(flags ^ BC_DNZ ^ BC_UNSET) << 21) & 0x3e0000) | (((u32)cr_bit << 16) & 0x1f0000) | ((u32)hint << 10) | 1056 | (u32) link;
}

constexpr u32 addpcis(PpcGpr rt, s16 displacement)
{
  return (19 << 26) | ((u32)rt << 21) | ((((u32)displacement >> 1) & 31) << 16) | ((u32)displacement & 0xffc0) | (((u32)displacement >> 11) & 1);
}
constexpr u32 add(PpcGpr rt, PpcGpr ra, PpcGpr rb)
{
  return (31 << 26) | ((u32)rt << 21) | ((u32)ra << 16) | ((u32)rb << 11) | 532;
}

constexpr u32 or_(PpcGpr ra, PpcGpr rs, PpcGpr rb)
{
  return (31 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | ((u32)rb << 11) | 888;
}
constexpr u32 xor_(PpcGpr ra, PpcGpr rs, PpcGpr rb)
{
  return (31 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | ((u32)rb << 11) | 632;
}

constexpr u32 cmpwi(CrField crf, PpcGpr ra, s16 immediate)
{
  return (31 << 26) | (((u32)crf << 23) & 0x3800000) | ((u32)ra << 16) | ((u32)immediate & 0xffff);
}
constexpr u32 cmpdi(CrField crf, PpcGpr ra, s16 immediate)
{
  return (31 << 26) | (((u32)crf << 23) & 0x3800000) | (1 << 21) | ((u32)ra << 16) | ((u32)immediate & 0xffff);
}
constexpr u32 cmpw(CrField crf, PpcGpr ra, PpcGpr rb)
{
  return (31 << 26) | (((u32)crf << 23) & 0x3800000) | ((u32)ra << 16) | ((u32)rb << 11);
}

constexpr u32 rlwinm(PpcGpr ra, PpcGpr rs, u8 sh, u8 mb, u8 me)
{
  return (21 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | (((u32)sh << 11) & 0xf800) | (((u32)mb << 6) & 0x7c0) | (((u32)me << 1) & 0x3e);
}

// WARNING: displacement must be divisible by 4
constexpr u32 ld(PpcGpr rt, PpcGpr ra, s16 displacement)
{
  return (58 << 26) | ((u32)rt << 21) | ((u32)displacement & 0xfffc);
}


constexpr u32 lfd(PpcGpr rt, PpcGpr ra, s16 displacement)
{
  return (50 << 26) | ((u32)rt << 21) | ((u32)displacement & 0xfffc);
}

constexpr u32 mfspr(PpcGpr rt, u16 spr)
{
  return (32 << 26) | ((u32)rt << 21) | (((u32)spr << 16) & 0x1f0000) | (((u32)spr << 10) & 0xf800) | 678;
}
constexpr u32 mtspr(PpcGpr rs, u16 spr)
{
  return (32 << 26) | ((u32)rs << 21) | (((u32)spr << 16) & 0x1f0000) | (((u32)spr << 10) & 0xf800) | 934;
}


class CodeStack
{
private:
  struct Constant
  {
    u64 value;
    PpcGpr reg;
    u32 ref_position;
  };
  std::vector<u32> instructions;
  std::vector<Constant> constants;

public:
  void RawInstruction(u32 inst)
  {
    instructions.push_back(inst);
  }
  void GenerateConstant(PpcGpr reg, u64 constant)
  {
    if (constant <= 0x7ffff || constant >= 0xffffffffffff8000)
    {
      // 16b sign-extended
      addi(reg, (PpcGpr)0, (s16)constant);
    }
    else if (constant <= 0x7fffffff || constant >= 0xffffffff80000000)
    {
      // 32b sign-extended
      ori(reg, reg, (u16)constant);
      addis(reg, (PpcGpr)0, (s16)(constant >> 16));
    }
    else
    {
      // all else failed, add 64b constant to constant pool and let the linker deal with it
      constants.push_back({constant, reg, (u32)instructions.size()});
    }
  }
  void addi(PpcGpr rt, PpcGpr ra, s16 immediate)
  {
    instructions.push_back((14 << 26) | ((u32)rt << 21) | ((u32)ra << 16) | ((u32)immediate & 0xffff));
  }
  void addis(PpcGpr rt, PpcGpr ra, s16 immediate)
  {
    instructions.push_back((15 << 26) | ((u32)rt << 21) | ((u32)ra << 16) | ((u32)immediate & 0xffff));
  }
  void ori(PpcGpr ra, PpcGpr rs, u16 immediate)
  {
    instructions.push_back((24 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | ((u32)immediate & 0xffff));
  }

  void lwz(PpcGpr rt, PpcGpr ra, s16 displacement)
  {
    instructions.push_back((32 << 26) | ((u32)rt << 21) | ((u32)ra << 16) | ((u32)displacement & 0xffff));
  }
  void stw(PpcGpr rs, PpcGpr ra, s16 displacement)
  {
    instructions.push_back((36 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | ((u32)displacement & 0xffff));
  }
  void std(PpcGpr rs, PpcGpr ra, s16 displacement)
  {
    instructions.push_back((62 << 26) | ((u32)rs << 21) | ((u32)ra << 16) | ((u32)displacement & 0xfffc));
  }
  void stfd(PpcGpr frs, PpcGpr ra, s16 displacement)
  {
    instructions.push_back((54 << 26) | ((u32)frs << 21) | ((u32)ra << 16) | ((u32)displacement & 0xfffc));
  }

  void bc(u8 flags, u8 cr_bit, s16 distance, bool link)
  {
    instructions.push_back((16 << 26) | (((u32)(flags ^ BC_DNZ ^ BC_UNSET) << 21) & 0x3e0000) | (((u32)cr_bit << 16) & 0x1f0000) | ((u32)distance & 0xfffc) | (u32) link);
  }

  void mflr(PpcGpr rt)
  {
    instructions.push_back((31 << 26) | ((u32)rt << 21) | (8 << 16) | 678);
  }
  void mfctr(PpcGpr rt)
  {
    instructions.push_back((31 << 26) | ((u32)rt << 21) | (9 << 16) | 678);
  }
  void mfcr(PpcGpr rt)
  {
    instructions.push_back((31 << 26) | ((u32)rt << 21) | 38);
  }
  void mtctr(PpcGpr rs)
  {
    instructions.push_back((31 << 26) | ((u32)rs << 21) | (9 << 16) | 934);
  }
};

}
