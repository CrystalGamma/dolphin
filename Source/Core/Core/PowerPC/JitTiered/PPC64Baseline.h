// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <optional>
#include <vector>

#include "Common/PPCEmitter.h"
#include "Core/PowerPC/Gekko.h"
#include "Core/PowerPC/PPCTables.h"
#include "Core/PowerPC/PowerPC.h"

class PPC64BaselineCompiler final : public PPCEmitter
{
public:
  PPC64BaselineCompiler() {}
  void Compile(u32 addr, const std::vector<UGeckoInstruction>& instructions);

private:
  enum JumpFlags
  {
    JUMP = 0,
    LINK = 1,
    SKIP = 2,
    JUMPSPR = 4,
  };

  struct Exit
  {
    FixupBranch branch;
    u32 address;
    s32 downcount;
    JumpFlags flags;
    u32 link_address;
  };

  struct JumpExit
  {
    FixupBranch branch;
    s32 downcount;
  };

  static constexpr s16 GPROffset(u32 i)
  {
    return s16(offsetof(PowerPC::PowerPCState, gpr) + 4 * i);
  }

  static constexpr s16 SPROffset(u32 i)
  {
    return s16(offsetof(PowerPC::PowerPCState, spr) + 4 * i);
  }

  void FallbackToInterpreter(UGeckoInstruction inst, GekkoOPInfo& opinfo);
  void BCX(UGeckoInstruction inst, GekkoOPInfo& opinfo);

  static constexpr GPR PPCSTATE = R30;
  static constexpr GPR TOC = R29;
  static constexpr GPR SCRATCH1 = R7;
  static constexpr GPR SCRATCH2 = R8;
  static constexpr GPR SAVED1 = R31;
  static constexpr GPR ARG1 = R3;
  static constexpr s16 OFF_PC = s16(offsetof(PowerPC::PowerPCState, pc));
  static constexpr s16 OFF_DOWNCOUNT = s16(offsetof(PowerPC::PowerPCState, downcount));
  static constexpr s16 OFF_EXCEPTIONS = s16(offsetof(PowerPC::PowerPCState, Exceptions));

  u32 address = 0;
  s32 downcount = 0;

  std::vector<Exit> jumps;
  std::vector<JumpExit> jmp_exits;
  std::vector<JumpExit> exc_exits;
  std::optional<JumpExit> float_check;
};
