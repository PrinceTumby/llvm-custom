//===- MipsR5900ShortLoopBugPass.cpp - Mips R5900 short loop bugfix pass -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// The R5900 has a hardware bug where a loop consisting of six or fewer
// instructions, including the branch delay slot, may only execute once or
// twice.
//
// This pass scans for short loops, and inserts nops to pad the basic block to
// at least seven instructions.
//===----------------------------------------------------------------------===//

#include "Mips.h"
#include "MipsInstrInfo.h"
#include "MipsSubtarget.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mips-r5900-short-loop-fix"

using namespace llvm;

namespace {

class MipsShortLoopBugFix : public MachineFunctionPass {
public:
  MipsShortLoopBugFix() : MachineFunctionPass(ID) {}

  StringRef getPassName() const override {
    return "Mips R5900 short loop bugfix";
  }

  MachineFunctionProperties getRequiredProperties() const override {
    return MachineFunctionProperties().setNoVRegs();
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  static char ID;

private:
  bool fixShortLoopBB(MachineBasicBlock &MBB, const MipsInstrInfo &II);

  std::optional<unsigned int> loopsBacktoMBB(
      MachineBasicBlock *CurrentMBB,
      MachineBasicBlock *OriginalMBB,
      unsigned int CurrentInstructionCount
  );

  void padBeginning(MachineBasicBlock &MBB,
                    const MipsInstrInfo &II,
                    unsigned int NOOPsToAdd);
};

} // namespace

INITIALIZE_PASS(MipsShortLoopBugFix, "mips-r5900-short-loop-fix",
                "Mips R5900 short loop bugfix", false, false)

char MipsShortLoopBugFix::ID = 0;

bool MipsShortLoopBugFix::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "Start MipsR5900ShortLoopBugPass\n";);
  const MipsInstrInfo &II =
      *static_cast<const MipsInstrInfo *>(MF.getSubtarget().getInstrInfo());

  bool Modified = false;

  for (auto &MBB : MF)
    Modified |= fixShortLoopBB(MBB, II);

  return Modified;
}

bool MipsShortLoopBugFix::fixShortLoopBB(MachineBasicBlock &MBB,
                                         const MipsInstrInfo &II) {
  // Pad all basic blocks which loop back within seven instructions.
  if (auto CycleInstructions = loopsBacktoMBB(&MBB, &MBB, 0)) {
    assert(*CycleInstructions < 8 &&
        "Cycle contains fewer than 8 instructions");
    padBeginning(MBB, II, 8 - *CycleInstructions);
    return true;
  } else {
    return false;
  }
}

std::optional<unsigned int> MipsShortLoopBugFix::loopsBacktoMBB(
    MachineBasicBlock *CurrentMBB,
    MachineBasicBlock *OriginalMBB,
    unsigned int CurrentInstructionCount
) {
  auto NoDebugInstrs =
    instructionsWithoutDebug(CurrentMBB->begin(), CurrentMBB->end());
  unsigned int MBBNumInstructions =
    std::distance(NoDebugInstrs.begin(), NoDebugInstrs.end());
  unsigned int NewInstructionCount =
    CurrentInstructionCount + MBBNumInstructions;
  if (NewInstructionCount > 7) return {};
  for (MachineBasicBlock *SuccMBB : CurrentMBB->successors()) {
    if (SuccMBB == OriginalMBB) return NewInstructionCount;
    if (auto InstrCount = loopsBacktoMBB(SuccMBB, OriginalMBB, NewInstructionCount)) {
      return *InstrCount;
    }
  }
  return {};
}

void MipsShortLoopBugFix::padBeginning(MachineBasicBlock &MBB,
                                       const MipsInstrInfo &II,
                                       unsigned int NOPsToAdd) {
  if (MBB.empty()) {
    assert(false && "Basic block should not be empty");
    // const DebugLoc DL = MBB.findDebugLoc(MBB.instr_begin());
    // for (unsigned int i = 0, e = NOPsToAdd; i < e; i++) {
    //   auto I = MBB.instr_front();
    //   const DebugLoc &DL = I.getDebugLoc();
    //   BuildMI(MBB, I, DL, II.get(Mips::NOP));
    // }
  } else {
    for (unsigned int i = 0, e = NOPsToAdd; i < e; i++) {
      MachineInstr &I = MBB.instr_front();
      const DebugLoc &DL = I.getDebugLoc();
      BuildMI(MBB, I, DL, II.get(Mips::NOP));
    }
  }
}

FunctionPass *llvm::createMipsShortLoopBugPass() { return new MipsShortLoopBugFix(); }
