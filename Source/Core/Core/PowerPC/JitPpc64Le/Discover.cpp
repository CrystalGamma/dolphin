#include "InstTable.h"

constexpr u8 inst2_imm[] = {
  14,  // addi
  12,  // addic
  13,  // addic.
  28,  // andi
  29,  // andis.
  7,  // mulli
  24,  // ori
  25,  // oris
};

// values for bits 22..30 when PO is 31
constexpr u16 alu_reg_xo[] = {
  8,  // subfc
  10, // addc
  40, // subf
  42,  // add
  136, // subfe
  138,  // adde
  235,  // mullw
  459, // divwu
  491, // divw
};
// bitset of bits 22..26 when PO is 31, bit 21 is 0 and bits 27..30 are 0b1100
constexpr u32 bitops_reg_xo = 0x2a0a8a
// 0010 (nand) 1010 (or, orc)
// 0000 1010 (xor, eqv)
// 1000 (nor) 1010 (andc, and)

// bits 1..5 when bit 0 is 0
constexpr u8 load_xo[] = {
  16, // lwz(u)
  17, // lbz(u)
  20, // lhz(u)
  21, // lha(u)
  24, // lfs(u)
  25, // lfd(u)
};
constexpr u8 store_xo[] = {
  18, // stw(u)
  19, // stb(u)
  22, // sth(u)
  26, // stfs(u)
  27, // stfd(u)
};

// bitset for bits 0..5
constexpr u64 SAFE_PO = 0x00ff3fff3f00f180;
// 0000 0000 (various)
// 1111 (float store) 1111 (float load)
// 0011 (string load/store, halfword store) 1111 (halfword load)
// 1111 (store byte/word) 1111 (load byte/word and zero)
// 0011 (various, bit-and) 1111 (xor, or)
// 1011 (rotate) 0000 (branching)
// 1111 (add immediate) 0001 (compare, subfic)
// 1000 (multiply) 0000 (trap)

// bitset for bits 22..25 when PO is 19, bit 21 is 0 and bits 26..31 are 0b000010
constexpr u16 safe_cr_bitops = 0x63d2;
// 0110 (orc, or) 0011 (and, eqv)
// 1101 (xor, nand, andc) 0010 (nor)

enum DiscoverClass {
  Safe,
  Branch,
  Bail
}

DiscoverClass classify_inst_discover(UGeckoInstruction inst) {
  if (inst.OPCD == 16 || inst.OPCD == 18)
  {
    return Branch;
  }
  if (((SAFE_PO >> inst.OPCD) & 1) == 1)
  {
    // loads, stores and ops with immediate operands
    return inst.OPCD >= 48 ? Float : Safe;
  }
  if (inst.OPCD == 19)
  {
    if ((inst.hex & 0x43f) == 2 && ((safe_cr_bitops >> ((inst.hex >> 6) & 15)) & 1) == 1)
    {
      // CR bit operations
      return Safe;
    }
    return Bail;
  }
  if (inst.OPCD == 31)
  {
    if ((inst.hex & 0x20f) == 12)
    {
      if (((bitops_reg_xo >> ((inst >> 4) & 31)) & 1) == 1)
      {
        // GPR bit operations
        return Safe;
      }
      return Bail;
    }
    u16 xo = (inst.hex >> 1) & 0x1ff;
    for (int i = 0; i < (sizeof(alu_reg_xo) / sizeof(u16)); i ++)
    {
      if (alu_reg_xo[i] == xo)
      {
        // ALU operations
        return Safe;
      }
      if (alu_reg_xo[i] > xo)
      {
        break;
      }
    }
    if (inst.OPCD == 19 && ((inst.hex & 0xfffe) == 32 || (inst.hex & 0xfffe) == 1056))
    {
      return Branch;
    }
    return Bail;
  }
  if ((inst.OPCD & 61) == 61)
  {
    u8 xo = (inst >> 1) & 31;
    bool op1 = (inst & (31 << 16)) != 0;
    bool op2 = (inst & (31 << 11)) != 0;
    bool op3 = (inst & (31 << 6)) != 0;
    // TODO: check zero-ness of operands for float operations
  }
  u16 xo = inst.hex & 0x3ff;
  if ((inst.OPCD == 31 || inst.OPCD == 63) && (xo == 0 || xo == 64) && ((inst >> 21) & 3 == 0))
  {
    // register-register compare operations
    return inst.OPCD == 63 ? Float : Safe;
  }
  return Bail;
}

// this flag is used for the work list inside Discover, and are translated to appropriate settings in the block struct
constexpr u32 ENTRY_POINT = 1;

// in the context of this JIT, a block is a sequence of instructions that are contiguous in emulated memory
// it begins wherever a not-compiled yet jump leads and ends with one of the following "bail" conditions:
// 1) an unconditional jump or other instruction that will with high probability change the PC
// 2) an instruction that is emulated using the interpreter, and may cause the PC and/or privileged state (MSR, …) to change
// 3) the end of an MMU block (BAT configuration if MSR.IR=0, page if MSR.IR=1)
// Float instructions (which may cause exceptions) or conditional branches and traps do not end a block.
struct Block
{
  u32 start;
  u32 length;
  // contains all the entry points (points in the block where control flow may enter from outside (this includes returns) in the block
  // The value describes whether the entry point is exposed to the dispatcher.
  // Not giving an entry point to the dispatcher has the advantage that we can optimize things like MSR.FP checks
  std::map<u32, bool> entry_points;

  Block(u32 start_, u32 len) : start(start_), length(len) {}
};

std::pair<std::vector<Block>, std::map<u32, u32>> Discover(u32 start_addr)
{
  std::vector<Block> res;
  // maps entry address to block in the result vector
  std::map<u32, u32> entry_points;
  // contains jump addresses to investigate (second member is flags)
  std::vector<std::pair<u32, u32>> work_list;
  // this limits how many addresses behind a branch-and-link instructions will be investigated in this invocation of Discover
  int function_budget = 10;
  // this limits how many blocks will be investigated by this invocation of Discover
  int block_budget = 1000;

  work_list.push_back(std::make_pair(start_addr, 0));
  while (!work_list.empty()) {
    auto job = work_list.back();
    u32 addr = job.first;
    u32 flags = job.second;
    auto find = entry_points.upper_bound(addr);
    if (find != entry_points.begin())
    {
      // we might be compiling a loop
      find--;
      u32 offset = (addr - find->first) / 4;
      if (find->second.len() > offset)
      {
        // we are in the middle of the block, insert the entry point if necessary
        auto entry_find = find->second.entry_points.find(offset);
        if (entry_find != find->second.entry_points.end())
        {
          if ((flags & ENTRY_POINT) != 0)
          {
            entry_find -> second = true;
          }
        }
        else
        {
          find->second.entry_points.insert(offset, (flags & ENTRY_POINT) != 0);
        }
        continue;
      }
    }
    if (block_cache.Dispatch(addr))
    {
      // don't compile the same code twice if we can avoid it
      continue;
    }
    work_list.pop_back();
    u32 *inst_ptr = nullptr; /* TODO: translate address */
    u32 *stop = nullptr;

    Block *block = nullptr;
    entry_points.insert(addr, res.length());
    block = &res.emplace_back(addr, 0, flags, 0);
    if (flags & ENTRY_POINT != 0)
    {
      flags &= ~ENTRY_POINT;
      block->entry_points.insert(0, true);
    }
    u32 counter = 0;
    u32 float_safe_at = 0;
    while (inst_ptr != stop) {
      counter += 1;
      UGeckoInstruction instruction(swap32(*inst_ptr));
      DiscoverClass classification = classify_inst_discover(instruction);
      bool end_block = false;
      bool entry_point = false;
      if (classification == Branch)
      {
        u32 target = 0;
        bool call;
        if (instruction.OPCD == 18)
        {
          // unconditional jumps end block, calls don't
          call = instruction.LK == 1;
          end_block = !call;
          if (instruction.AA == 0)
          {
            // TODO fix Gekko.h
            target = addr + 4*(counter + instruction.LI - (1 << 23));
          }
          else
          {
            target = (instruction.LI - (1 << 23)) * 4;
          }
        }
        else if (instruction.OPCD == 16)
        {
          call = instruction.LK;
          // conditional branches don't end the block, unless always taken
          end_block = (instruction.hex & 0x2800000) == 2800000;
          if (instruction.AA == 0)
          {
            target = addr + 4*(counter + instruction.BD - (1 << 13));
          }
          else
          {
            target = 4*(instruction.BD - (1 << 13));
          }
        }
        else if (instruction.OPCD == 19)
        {
          // end block if always taken
          end_block = (instruction.hex & 0x2800000) == 2800000;
          call = instruction.LK;
          // since bclr and bcctr work on registers, we don't know their targets
          // in fact, bclr is the typical return instruction, we probably already have it
        }
        else
        {
          _assert_msg_(DYNA_REC, false, "unknown branch instruction");
        }
        if (target != 0 && block_budget > 0)
        {
          if (call && function_budget > 0)
          {
            work_list.push_back(std::make_pair(target, ENTRY_POINT));
            function_budget -= 1;
            block_budget -= 1;
          }
          else if (!call)
          {
            work_list.push_back(std::make_pair(target, flags));
            block_budget -= 1;
          }
        }
      }
      if (classification == Bail)
      {
        end_block = true;
      }
      if (entry_point && (entry_points.count(addr + 4*counter) > 0 || block_cache.Dispatch(addr)))
      {
        // we already had this entry point, don't get caught in a loop
        entry_point = false;
        end_block = true;
      }
      if (entry_point)
      {
        entry_points.insert(addr + 4*counter, res.length() - 1);
        block->entry_points.insert(addr);
      }
      if (end_block)
      {
        block->length = counter;
        break;
      }
      // do stuff
      instruction += 1;
    }
  }
  return std::make_pair(res, entry_points);
}
