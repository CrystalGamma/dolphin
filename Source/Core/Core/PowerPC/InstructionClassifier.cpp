#include "Core/PowerPC/InstructionClassifier.h"

int InstructionClassifier::cycles(UGeckoInstruction inst)
{
  switch (inst.OPCD)
  {
  case 31:
    if ((inst.hex & 0x3e) == 0x16)
    { // integer multiply/divide
      if ((inst.hex & 0x300) == 0x300)
      { // divw(u)(o)(.)
        return 40;
      }
      if ((inst.hex & 0x740) == 0 || (inst.hex & 0x3c0) == 0x1c0)
      { // mulhw(u)(.), mullw(o)(.)
        return 5;
      }
    }
    else if ((inst.hex & 0x3f) == 0x2c)
    { // cache instructions
      switch ((inst.hex >> 6) & 63)
      {
      case 7: // dcbtst
      case 8: // dcbt
        return 2;
      case 1: // dcbst
      case 2: // dcbf
      case 14: // dcbi
      case 23: // dcba
        return 5;
      case 30: // icbi
        return 4;
      }
    }
    else if (inst.SUBOP10 == 595 || inst.SUBOP10 == 659 || inst.SUBOP10 == 598)
    { // mfsr(in), sync
      return 3;
    }
    else if (inst.SUBOP10 == 467 || inst.SUBOP10 == 4)
    { // mtspr, tw
      return 2;
    }
    return 1;
  case 2: return 2; // sc
  case 7: return 3; // mulli
  case 46:  // lmw
  case 47:  // stmw
    return 11;
  case 4:
    switch (inst.hex & 0x3e)
    {
    case 36: return 17; // ps_div(.)
    case 52: return 2; // ps_rsqrte
    default: return 1;
    }
  case 59:
    if (inst.SUBOP10 == 18)
    { // fdivs(.)
      return 17;
    }
    return 1;
  case 63:
    switch (inst.SUBOP10)
    {
    case 18: return 31; // fdiv(.)
    case 70:  // mtfsb0(.)
    case 38:  // mtfsb1(.)
    case 134: // mtfsfi(.)
    case 711: // mtfsf(.)
      return 3;
    default: return 1;
    }
}

bool InstructionClassifier::Redispatch(UGeckoInstruction inst)
{
  return inst.OPCD == 19 && (inst.SUBOP10 == 150 || inst.SUBOP10 == 18)  // isync, rfid
    || inst.OPCD == 31 && inst.SUBOP10 == 146// mtmsr
    || inst.OPCD == 17 && (inst & 2) // sc
}
