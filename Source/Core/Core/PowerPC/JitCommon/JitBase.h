// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <cstddef>
#include <map>
#include <unordered_set>

#include "Common/CommonTypes.h"
#include "Common/x64Emitter.h"
#include "Core/ConfigManager.h"
#include "Core/MachineContext.h"
#include "Core/PowerPC/CPUCoreBase.h"
#include "Core/PowerPC/JitCommon/JitAsmCommon.h"
#include "Core/PowerPC/JitCommon/JitCache.h"
#include "Core/PowerPC/JitInterface.h"
#include "Core/PowerPC/PPCAnalyst.h"
#include "Core/PowerPC/Profiler.h"

//#define JIT_LOG_GENERATED_CODE  // Enables logging of generated code
//#define JIT_LOG_GPR             // Enables logging of the PPC general purpose regs
//#define JIT_LOG_FPR             // Enables logging of the PPC floating point regs

// Use these to control the instruction selection
// #define INSTRUCTION_START FallBackToInterpreter(inst); return;
// #define INSTRUCTION_START PPCTables::CountInstruction(inst);
#define INSTRUCTION_START

#define FALLBACK_IF(cond)                                                                          \
  do                                                                                               \
  {                                                                                                \
    if (cond)                                                                                      \
    {                                                                                              \
      FallBackToInterpreter(inst);                                                                 \
      return;                                                                                      \
    }                                                                                              \
  } while (0)

#define JITDISABLE(setting)                                                                        \
  FALLBACK_IF(SConfig::GetInstance().bJITOff || SConfig::GetInstance().setting)

class JitBase : public CPUCoreBase
{
public:
  /// handle invalid memory accesses and return true if caused by fastmem, return false otherwise.
  /// should be safe to execute inside a signal handler
  virtual bool HandleFault(uintptr_t access_address, SContext* ctx) = 0;
  /// handle stack overflows while in JIT code
  virtual bool HandleStackFault() { return false; }
  /// clear all JIT caches and code spaces; do not call while potentially inside JIT code!
  virtual void ClearCache() = 0;
  /// invalidate dispatch caches, but do not clear code spaces; must be safe to call inside JIT code
  virtual void ClearSafe() = 0;
  /// remove any blocks intersecting the specified range from any JIT caches
  virtual void InvalidateICache(u32 address, u32 size, bool forced) = 0;
  /// find a block of host machine code that emulates the instruction at the specified address.
  /// return 0 if a block was found, and fill out address with the effective address of the entry of
  /// the block, code with the entry point of the host code and code_size with the size of the block
  /// in host machine code. if no block was found, return 2 and set code_size to 0. if getting host
  /// machine code block is unsupported, return 1 and set code_size to 0.
  virtual int GetHostCode(u32* address, const u8** code, u32* code_size)
  {
    *code_size = 0;
    return 1;
  }
  virtual void EnableProfiling(bool enabled) {}
  virtual void GetProfileResults(Profiler::ProfileStats* prof_stats) {}
  virtual void CompileExceptionCheck(JitInterface::ExceptionType) {}
};

class JitBaseBlockCache;

class JitCommonBase : public JitBase
{
  friend class JitBaseBlockCache;

protected:
  struct JitOptions
  {
    bool enableBlocklink;
    bool optimizeGatherPipe;
    bool accurateSinglePrecision;
    bool fastmem;
    bool memcheck;
    bool profile_blocks;
  };
  struct JitState
  {
    u32 compilerPC;
    u32 blockStart;
    int instructionNumber;
    int instructionsLeft;
    int downcountAmount;
    u32 numLoadStoreInst;
    u32 numFloatingPointInst;
    // If this is set, we need to generate an exception handler for the fastmem load.
    u8* fastmemLoadStore;
    // If this is set, a load or store already prepared a jump to the exception handler for us,
    // so just fixup that branch instead of testing for a DSI again.
    bool fixupExceptionHandler;
    Gen::FixupBranch exceptionHandler;

    bool assumeNoPairedQuantize;
    std::map<u8, u32> constantGqr;
    bool firstFPInstructionFound;
    bool isLastInstruction;
    int skipInstructions;
    bool carryFlagSet;
    bool carryFlagInverted;

    bool generatingTrampoline = false;
    u8* trampolineExceptionHandler;

    bool mustCheckFifo;
    int fifoBytesSinceCheck;

    PPCAnalyst::BlockStats st;
    PPCAnalyst::BlockRegStats gpa;
    PPCAnalyst::BlockRegStats fpa;
    PPCAnalyst::CodeOp* op;

    JitBlock* curBlock;

    std::unordered_set<u32> fifoWriteAddresses;
    std::unordered_set<u32> pairedQuantizeAddresses;
    std::unordered_set<u32> noSpeculativeConstantsAddresses;
  };

  PPCAnalyst::CodeBlock code_block;
  PPCAnalyst::CodeBuffer m_code_buffer;
  PPCAnalyst::PPCAnalyzer analyzer;
  JitOptions jo{};
  JitState js{};

  bool CanMergeNextInstructions(int count) const;

  void UpdateMemoryOptions();

public:
  JitCommonBase();
  ~JitCommonBase() override;

  virtual JitBaseBlockCache* GetBlockCache() = 0;

  virtual void Jit(u32 em_address) = 0;

  virtual const CommonAsmRoutinesBase* GetAsmRoutines() = 0;
  virtual void InvalidateICache(u32 address, u32 size, bool forced)
  {
    GetBlockCache()->InvalidateICache(address, size, forced);
  }
  void ClearSafe() { GetBlockCache()->Clear(); }
  virtual int GetHostCode(u32* address, const u8** code, u32* code_size);
  virtual void GetProfileResults(Profiler::ProfileStats* prof_stats);
  virtual void CompileExceptionCheck(JitInterface::ExceptionType);
  virtual void EnableProfiling(bool enabled) { jo.profile_blocks = enabled; }

  static const u8* Dispatch(JitCommonBase& jit);

  static constexpr std::size_t code_buffer_size = 32000;
};

void JitTrampoline(JitCommonBase& jit, u32 em_address);
