// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "Common/BitUtils.h"
#include "Common/CommonTypes.h"

class PPCEmitter
{
public:
  using FixupBranch = size_t;

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
    SPR_XER = 1,
    SPR_LR = 8,
    SPR_CTR = 9
  };

  /// CRF values are always left-aligned in their 5-bit instruction field,
  /// and this lets us express CRB values as u32(CRx) + u32(EQ) for example
  enum CRF
  {
    CR0 = 0,
    CR1 = 4,
    CR2 = 8,
    CR3 = 12,
    CR4 = 16,
    CR5 = 20,
    CR6 = 24,
    CR7 = 28
  };

  /// within each field
  enum CRB
  {
    LT = 0,
    GT = 1,
    EQ = 2,
    SO = 3
  };

  /// constants for the BO field in conditional branches
  /// hint bits are ignored here
  enum BranchMode
  {
    BR_ALWAYS = 20,
    BR_TRUE = 12,
    BR_FALSE = 4,
    BR_DEC_NZ = 16,
    BR_DEC_ZERO = 18,
    BR_DEC_NZ_TRUE = 8,
    BR_DEC_ZERO_TRUE = 10,
    BR_DEC_NZ_FALSE = 0,
    BR_DEC_ZERO_FALSE = 2,
  };

  /// hints for bclr and bcctr
  enum BranchHint
  {
    HINT_RETURN = 0,
    HINT_LR_PREDICTABLE = 1,
    HINT_CTR_PREDICTABLE = 0,
    HINT_UNPREDICTABLE = 3
  };

  /// flags for direct branches (bx, bcx)
  enum BranchFlags
  {
    BR_NORMAL = 0,
    BR_LINK = 1,
    BR_ABSOLUTE = 2
  };

  enum LSUpdate
  {
    NO_UPDATE = 0,
    UPDATE = 1
  };

  enum TrapCondition
  {
    TRAP_ALWAYS = 31,
    TRAP_LT = 16,
    TRAP_GT = 8,
    TRAP_EQ = 4,
    TRAP_ULT = 2,
    TRAP_UGT = 1
  };

  enum CompareSize
  {
    CMP_WORD = 0,
    CMP_DWORD = 1
  };

  enum Record
  {
    NO_RC = 0,
    RC = 1
  };

  // === instruction form helpers ===
  // these functions take their fields in encoding order, most other functions take them in
  // assembler order (destination-first)
  void DFormInstruction(u32 opcode, GPR r1, GPR r2, u16 imm)
  {
    instructions.push_back((opcode << 26) | (u32(r1) << 21) | (u32(r2) << 16) | u32(imm));
  }
  void DFormInstructionSigned(u32 opcode, GPR r1, GPR r2, s16 imm)
  {
    DFormInstruction(opcode, r1, r2, Common::BitCast<u16>(imm));
  }

  void XFormInstruction(u32 opcode, GPR r1, GPR r2, GPR r3, u32 subop10)
  {
    instructions.push_back((opcode << 26) | (u32(r1) << 21) | (u32(r2) << 16) | (u32(r3) << 11) |
                           (subop10 << 1));
  }
  void XLFormInstruction(u32 opcode, u32 bt, u32 ba, u32 bb, u32 subop10)
  {
    instructions.push_back((opcode << 26) | ((bt & 31) << 21) | ((ba & 31) << 16) |
                           ((bb & 31) << 11) | (subop10 << 1));
  }
  void MFormInstruction(u32 opcode, GPR rs, GPR ra, u32 sh, u32 mb, u32 me)
  {
    instructions.push_back((opcode << 26) | (u32(rs) << 21) | (u32(ra) << 16) | ((sh & 31) << 11) |
                           ((mb & 31) << 6) | ((me & 31) << 1));
  }

  // === integer immediate instructions ===
  // do not call on register 0 (use LoadSignedImmediate instead, for clarity)
  void ADDI(GPR rt, GPR ra, s16 imm) { DFormInstructionSigned(14, rt, ra, imm); }
  // do not call on register 0 (use LoadSignedImmediate instead, for clarity)
  void ADDIS(GPR rt, GPR ra, s16 imm) { DFormInstructionSigned(15, rt, ra, imm); }

  void ORI(GPR ra, GPR rs, u16 imm) { DFormInstruction(24, rs, ra, imm); }
  void ORIS(GPR ra, GPR rs, u16 imm) { DFormInstruction(25, rs, ra, imm); }

  void XORI(GPR ra, GPR rs, u16 imm) { DFormInstruction(26, rs, ra, imm); }
  void XORIS(GPR ra, GPR rs, u16 imm) { DFormInstruction(27, rs, ra, imm); }

  void ANDI_Rc(GPR ra, GPR rs, u16 imm) { DFormInstruction(28, rs, ra, imm); }
  void ANDIS_Rc(GPR ra, GPR rs, u16 imm) { DFormInstruction(29, rs, ra, imm); }

  void MULLI(GPR rt, GPR ra, s16 imm) { DFormInstructionSigned(7, rt, ra, imm); }

  void RLWINM(GPR ra, GPR rs, u32 sh, u32 mb, u32 me) { MFormInstruction(21, rs, ra, sh, mb, me); }
  void SRAWI(GPR ra, GPR rs, u32 sh)
  {
    XFormInstruction(31, rs, ra, static_cast<GPR>(sh & 31), 824);
  }

  void LoadSignedImmediate(GPR rt, s32 imm)
  {
    if (imm <= 0x7fff && imm >= -0x8000)
    {
      // imm is a 16-bit signed value
      ADDI(rt, R0, imm);
    }
    else
    {
      // use the 'performance-optimized instruction sequence' as per the spec
      // (even though most implementations, e. g. POWER9, don't do fusing)
      u16 lower_half = u16(Common::BitCast<u32>(imm) & 0xffff);
      ADDIS(rt, R0, static_cast<s16>(imm / (1 << 16)));
      if (lower_half)
      {
        ORI(rt, rt, lower_half);
      }
    }
  }
  void LoadUnsignedImmediate(GPR rt, u32 imm)
  {
    if (imm < 0x7fffffff)
    {
      LoadSignedImmediate(rt, s32(imm));
    }
    else if (!(imm & 0x8000))
    {
      LoadSignedImmediate(rt, s16(imm & 0x7fff));
      ORIS(rt, rt, u16(imm >> 16));
    }
    else
    {
      LoadSignedImmediate(rt, 0);
      // use the 'performance-optimized instruction sequence' as per the spec
      // (even though most implementations, e. g. POWER9, don't do fusing)
      ORIS(rt, rt, u16(imm >> 16));
      if (imm & 0xffff)
      {
        ORI(rt, rt, u16(imm & 0xffff));
      }
    }
  }
  void AddSImm32(GPR rt, GPR ra, s32 imm)
  {
    if (imm == 0)
    {
      if (ra != rt)
      {
        MoveReg(rt, ra);
      }
      return;
    }
    u32 repr = Common::BitCast<u32>(imm);
    u16 upper_half_u = u16(repr >> 16);
    s16 lower_half = Common::BitCast<s16>(u16(repr & 0xffff));
    // account for sign extension
    if (lower_half < 0)
    {
      upper_half_u += 1;
    }
    if (upper_half_u != 0)
    {
      ADDIS(rt, ra, Common::BitCast<s16>(upper_half_u));
    }
    if (lower_half != 0)
    {
      ADDI(rt, upper_half_u ? rt : ra, lower_half);
    }
  }

  // === reg-reg integer instructions ===
  void OR(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 444); }
  void AND(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 28); }
  void XOR(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 316); }
  void NAND(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 476); }
  void NOR(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 124); }
  void EQV(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 284); }
  void ANDC(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 60); }
  void ORC(GPR ra, GPR rs, GPR rb) { XFormInstruction(31, rs, ra, rb, 412); }

  void ADD(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 266); }
  void SUBF(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 40); }

  void MoveReg(GPR rt, GPR rs) { OR(rt, rs, rs); }

  // === compare ===
  void CMPLI(CRF bf, CompareSize l, GPR ra, u16 imm)
  {
    DFormInstruction(10, static_cast<GPR>(bf + l), ra, imm);
  }
  void CMPL(CRF bf, CompareSize l, GPR ra, GPR rb)
  {
    XFormInstruction(31, static_cast<GPR>(bf + l), ra, rb, 32);
  }
  void CMPI(CRF bf, CompareSize l, GPR ra, u16 imm)
  {
    DFormInstruction(11, static_cast<GPR>(bf + l), ra, imm);
  }
  void CMP(CRF bf, CompareSize l, GPR ra, GPR rb)
  {
    XFormInstruction(31, static_cast<GPR>(bf + l), ra, rb, 0);
  }

  // === CR operations ===
  void CRAND(u32 bt, u32 ba, u32 bb) { XLFormInstruction(19, bt, ba, bb, 257); }
  void CRNOR(u32 bt, u32 ba, u32 bb) { XLFormInstruction(19, bt, ba, bb, 33); }
  void CRANDC(u32 bt, u32 ba, u32 bb) { XLFormInstruction(19, bt, ba, bb, 129); }

  // === ≤32-bit load/store ===
  void LWZ(GPR rt, GPR ra, s16 disp) { DFormInstructionSigned(32, rt, ra, disp); }
  void LWZU(GPR rt, GPR ra, s16 disp) { DFormInstructionSigned(33, rt, ra, disp); }
  void STW(GPR rs, GPR ra, s16 disp) { DFormInstructionSigned(36, rs, ra, disp); }
  void STWU(GPR rs, GPR ra, s16 disp) { DFormInstructionSigned(37, rs, ra, disp); }

  void LBZX(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 87); }
  void STBX(GPR rs, GPR ra, GPR rb) { XFormInstruction(31, rs, ra, rb, 215); }

  void LHBRX(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 790); }
  void STHBRX(GPR rs, GPR ra, GPR rb) { XFormInstruction(31, rs, ra, rb, 918); }

  void LWBRX(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 534); }
  void STWBRX(GPR rs, GPR ra, GPR rb) { XFormInstruction(31, rs, ra, rb, 662); }

  // === branch ===
  // addr must fit into 26 bits (including sign)
  FixupBranch B(BranchFlags flags = BR_NORMAL, s32 addr = 0)
  {
    instructions.push_back((18u << 26) | (Common::BitCast<u32>(addr) & 0x3fffffc) |
                           (u32(flags) & 3));
    return instructions.size() - 1;
  }
  // crb < 32 obviously
  FixupBranch BC(BranchMode bo, u32 crb, BranchFlags flags = BR_NORMAL, s16 addr = 0)
  {
    instructions.push_back((16u << 26) | (u32(bo) << 21) | ((crb & 31) << 16) |
                           (u32(Common::BitCast<u16>(addr)) & 0xfffc) | (u32(flags) & 3));
    return instructions.size() - 1;
  }
  void BCLR(BranchMode bo = BR_ALWAYS, u32 crb = 0, BranchHint hint = HINT_RETURN,
            bool link = false)
  {
    instructions.push_back((19u << 26) | (static_cast<u32>(bo) << 21) | ((crb & 31) << 16) |
                           static_cast<u32>(hint) << 11 | (16u << 1) | static_cast<u32>(link));
  }
  void BCCTR(BranchMode bo = BR_ALWAYS, u32 crb = 0, BranchHint hint = HINT_CTR_PREDICTABLE,
             bool link = true)
  {
    instructions.push_back((19u << 26) | (u32(bo) << 21) | ((crb & 31) << 16) | u32(hint) << 11 |
                           (528u << 1) | u32(link));
  }

  void Relocate(FixupBranch branch, s32 delta)
  {
    u32 inst = instructions.at(branch);
    u32 opcode = inst >> 26;
    if (opcode == 16)
    {
      instructions[branch] = (inst & 0xffff0003) | ((inst + Common::BitCast<u32>(delta)) & 0xfffc);
    }
    else if (opcode == 18)
    {
      instructions[branch] =
          (inst & 0xfc000003) | ((inst + Common::BitCast<u32>(delta)) & 0x3fffffc);
    }
  }
  void SetBranchTarget(FixupBranch branch) { Relocate(branch, (instructions.size() - branch) * 4); }

  // === system registers ===
  // the order of the two 5-bit halves of the spr field is reversed in the encoding
  void MTSPR(SPR spr, GPR rs)
  {
    instructions.push_back((31u << 26) | (u32(rs) << 21) | ((u32(spr) & 31) << 16) |
                           ((u32(spr) & 0x3e0) << 6) | (467u << 1));
  }
  void MFSPR(GPR rt, SPR spr)
  {
    instructions.push_back((31u << 26) | (u32(rt) << 21) | ((u32(spr) & 31) << 16) |
                           ((u32(spr) & 0x3e0) << 6) | (339u << 1));
  }

  // === misc ===
  void TW(TrapCondition to = TRAP_ALWAYS, GPR ra = R0, GPR rb = R0)
  {
    instructions.push_back((31u << 26) | (u32(to) << 21) | (u32(ra) << 16) | (u32(rb) << 11) |
                           (4u << 1));
  }
  void TWI(TrapCondition to, GPR ra, s16 imm)
  {
    DFormInstructionSigned(3, static_cast<GPR>(to), ra, imm);
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

  void LDX(GPR rt, GPR ra, GPR rb) { XFormInstruction(31, rt, ra, rb, 21); }
  void STDX(GPR rs, GPR ra, GPR rb) { XFormInstruction(31, rs, ra, rb, 149); }

  void EXTSW(GPR ra, GPR rs) { XFormInstruction(31, rs, ra, R0, 986); }

  void RLDICL(GPR ra, GPR rs, u32 sh, u32 mb, Record rc = NO_RC)
  {
    instructions.push_back((30u << 26) | (u32(rs) << 21) | (u32(ra) << 16) | ((sh & 31) << 11) |
                           ((mb & 31) << 6) | (mb & 32) | (0 << 2) | ((sh & 32) >> 4) | u32(rc));
  }

  std::vector<u32> instructions;
};
