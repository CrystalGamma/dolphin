// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "Common/BitUtils.h"
#include "Common/CommonTypes.h"

namespace PPCGen
{
enum GPR
{
  R0 = 0,
  R1 = 1,
  R2 = 2,
  R3 = 3,
  R4 = 4,
  R5 = 5,
  R6 = 6,
  R7 = 7,
  R8 = 8,
  R9 = 9,
  R10 = 10,
  R11 = 11,
  R12 = 12,
  R13 = 13,
  R14 = 14,
  R15 = 15,
  R16 = 16,
  R17 = 17,
  R18 = 18,
  R19 = 19,
  R20 = 20,
  R21 = 21,
  R22 = 22,
  R23 = 23,
  R24 = 24,
  R25 = 25,
  R26 = 26,
  R27 = 27,
  R28 = 28,
  R29 = 29,
  R30 = 30,
  R31 = 31
};

enum SPR
{
  XER = 1,
  LR = 8,
  CTR = 9
}

/// constants for the BO field in conditional branches
/// hint bits are ignored here
enum BranchMode
{
  BRANCH_ALWAYS = 20,
  BRANCH_TRUE = 12,
  BRANCH_FALSE = 4,
  BRANCH_DEC_ZERO = 16,
  BRANCH_DEC_NZ = 18,
  BRANCH_DEC_ZERO_TRUE = 10,
  BRANCH_DEC_NZ_TRUE = 8,
  BRANCH_DEC_ZERO_FALSE = 2,
  BRANCH_DEC_NZ_FALSE = 0,
}

/// hints for bclr and bcctr
enum BranchHint
{
  HINT_RETURN = 0,
  HINT_LR_PREDICTABLE = 1,
  HINT_CTR_PREDICTABLE = 0,
  HINT_UNPREDICTABLE = 3
}

enum LSUpdate
{
  NO_UPDATE = 0,
  UPDATE = 1
}

class PPCEmitter
{
public:
  using FixupBranch = size_t;

  void DFormInstruction(u32 opcode, GPR r1, GPR r2, u16 imm)
  {
    instructions.push_back((opcode << 26) | (static_cast<u32>(r1) << 21) |
                           (static_cast<u32>(r2) << 16) | static_cast<u32>(imm));
  }
  void DFormInstructionSigned(u32 opcode, GPR r1, GPR r2, s16 imm)
  {
    DFormInstruction(opcode, r1, r2, Common::BitCast<u16>(imm));
  }

  // === integer immediate instructions ===
  // do not call on register 0 (use LoadSignedImmediate instead, for clarity)
  void ADDI(GPR rt, GPR rs, s16 imm) { DFormInstructionSigned(14, rt, rs, imm); }
  // do not call on register 0 (use LoadSignedImmediate instead, for clarity)
  void ADDIS(GPR rt, GPR rs, s16 imm) { DFormInstructionSigned(15, rt, rs, imm); }

  void ORI(GPR rt, GPR rs, u16 imm) { DFormInstruction(24, rt, rs, imm) }
  void ORIS(GPR rt, GPR rs, u16 imm) { DFormInstruction(24, rt, rs, imm) }

  void XORI(GPR rt, GPR rs, u16 imm) { DFormInstruction(26, rt, rs, imm) }
  void XORIS(GPR rt, GPR rs, u16 imm) { DFormInstruction(27, rt, rs, imm) }

  void ANDI_Rc(GPR rt, GPR rs, u16 imm) { DFormInstruction(28, rt, rs, imm) }
  void ANDIS_Rc(GPR rt, GPR rs, u16 imm) { DFormInstruction(29, rt, rs, imm) }

  void LoadSignedImmediate(GPR rt, s32 imm)
  {
    if (imm <= 0x7fff && imm >= -0x8000)
    {
      // imm is a 16-bit signed value
      ADDI(rt, R0, imm);
    }
    else if (imm > 0x7fff && imm <= 0xffff)
    {
      // imm is a 16-bit unsigned value
      ORI(rt, R0, static_cast<u16>(imm));
    }
    else
    {
      // use the 'performance-optimized instruction sequence' as per the spec
      // (even though most implementations, e. g. POWER9, don't do fusing)
      u16 lower_half = static_cast<u16>((imm < 0 ? -imm : imm) & 0xffff);
      ADDIS(rt, R0, static_cast<s16>(imm / (1 << 16)));
      ORI(rt, R0, lower_half);
    }
  }

  // === ≤32-bit load/store ===
  void LWZ(GPR rt, GPR ra, s16 disp) { DFormInstructionSigned(32, rt, ra, disp); }
  void STW(GPR rs, GPR ra, s16 disp) { DFormInstructionSigned(36, rs, ra, disp); }

  // === branch ===
  // addr must fit into 26 bits (including sign)
  void B(s32 addr, bool link, bool absolute = false)
  {
    instructions.push_back((18u << 26) | (Common::BitCast<u32>(addr) & 0x3fffffc) |
                           (static_cast<u32>(absolute) << 1) | static_cast<u32>(link));
  }
  // crb < 32 obviously
  void BC(BranchMode bo, u32 crb, s16 addr, bool link = false, bool absolute = false)
  {
    instructions.push_back((16u << 26) | (u32(bo) << 21) | ((crb & 31) << 16) | u32(Common::BitCast<u16>(addr) | (u32(absolute) << 1) | u32(link));
  }
  void BCLR(BranchMode bo = BRANCH_ALWAYS, u32 crb = 0, BranchHint hint = HINT_RETURN,
            bool link = false)
  {
    instructions.push_back((16u << 26) | (static_cast<u32>(bo) << 21) | ((crb & 31) << 16) |
                           static_cast<u32>(hint) << 11 | (16u << 1) | static_cast<u32>(link));
  }
  void BCCTR(BranchMode bo, u32 crb, BranchHint hint = HINT_CTR_PREDICTABLE, bool link = false)
  {
    instructions.push_back((16u << 26) | (u32(bo) << 21) | ((crb & 31) << 16) | u32(hint) << 11 |
                           (528u << 1) | u32(link));
  }

  FixupBranch ConditionalBranch(BranchMode bo, u32 crb)
  {
    instructions.push_back((16u << 26) | (static_cast<u32>(bo) << 21) | ((crb & 31) << 16));
    return instructions.size() - 1;
  }
  void SetBranchTarget(FixupBranch branch)
  {
    u32 opcode = instructions.at(branch) >> 26;
    if (opcode == 16)
    {
      instructions[branch] |= static_cast<s16>((instructions.size() - branch) & 0x1fff) << 2;
    }
    else if (opcode == 18)
    {
      instructions[branch] |= static_cast<s32>((instructions.size() - branch) & 0xfffff) << 2;
    }
  }

  // === system registers ===
  // the order of the two 5-bit halves of the spr field is reversed in the encoding
  void MTSPR(SPR spr, GPR rs)
  {
    instructions.push_back((31u << 26) | (static_cast<u32>(rs) << 21) | ((static_cast<u32>(spr) & 31) << 16) | ((static_cast<u32>(spr) & 0x3e0) << 6) | (467u << 1);
  }
  void MFSPR(GPR rt, SPR spr)
  {
    instructions.push_back((31u << 26) | (static_cast<u32>(rs) << 21) | ((static_cast<u32>(spr) & 31) << 16) | ((static_cast<u32>(spr) & 0x3e0) << 6) | (339u << 1);
  }

  // === 64-bit instructions ===

  // displacement must be divisible by 4
  void LD(GPR rt, GPR ra, s16 disp, LSUpdate update = NO_UPDATE)
  {
    DFormInstruction(58, rt, ra, Common::BitCast<u16>(disp) | u16(update));
  }
  // displacement must be divisible by 4
  void STD(GPR rs, GPR ra, s16 disp, LSUpdate update = NO_UPDATE)
  {
    DFormInstruction(62, rs, ra, Common::BitCast<u16>(disp) | u16(update));
  }

  std::vector<u32> instructions;
};
}
