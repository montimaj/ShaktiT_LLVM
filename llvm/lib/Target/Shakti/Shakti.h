//===-- Shakti.h - Top-level interface for Shakti representation -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Shakti back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_SHAKTI_H
#define LLVM_LIB_TARGET_SHAKTI_SHAKTI_H

#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class FunctionPass;
class ShaktiTargetMachine;

FunctionPass *createShaktiISelDag(ShaktiTargetMachine &TM);

} // namespace llvm

#endif
