//===-- ShaktiTargetInfo.cpp - Shakti Target Implementation -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "Shakti.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

Target llvm::TheShaktiTarget;

extern "C" void LLVMInitializeShaktiTargetInfo() {
  RegisterTarget<Triple::shakti> X(TheShaktiTarget, "shakti", "Shakti");
}
