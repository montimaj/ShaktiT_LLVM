//===--- ShaktiTargetObjectFile.h - Shakti Object Info ---------*- C++ ----*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file deals with any Shakti specific requirements on object files.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_SHAKTI_SHAKTITARGETOBJECTFILE_H
#define LLVM_TARGET_SHAKTI_SHAKTITARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class TargetMachine;

class ShaktiTargetObjectFile : public TargetLoweringObjectFileELF {
  void Initialize(MCContext &Ctx, const TargetMachine &TM) override;
};

} // end namespace llvm

#endif
