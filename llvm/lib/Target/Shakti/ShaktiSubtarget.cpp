//===--------- ShaktiSubtarget.cpp - Shakti Subtarget Information  ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the shakti specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "shakti-subtarget"

#include "ShaktiSubtarget.h"
#include "Shakti.h"
#include "ShaktiTargetMachine.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "ShaktiGenSubtargetInfo.inc"

using namespace llvm;

void ShaktiSubtarget::anchor() {}

ShaktiSubtarget::ShaktiSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS,
                               const ShaktiTargetMachine &TM)
    : ShaktiGenSubtargetInfo(TT, CPU, FS),
      InstrInfo(*this),
      TargetLowering(TM, *this),
      FrameLowering(*this) {

  std::string CPUName = CPU;

  // CPUName is empty when invoked from tools like llc
  if (CPUName.empty())
    CPUName = "shakti";

  InstrItins = getInstrItineraryForCPU(CPUName);

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
}
