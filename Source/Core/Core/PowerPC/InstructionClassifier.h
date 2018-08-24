#pragma once

#include "Core/PowerPC/Gekko.h"

namespace InstructionClassifier {
  /// estimated cycle count for the given instruction
  int Cycles(UGeckoInstruction);
  /// instructions which require the block to end; essentially anything that synchronizes the instruction cache with memory
  bool Redispatch(UGeckoInstruction);
}
