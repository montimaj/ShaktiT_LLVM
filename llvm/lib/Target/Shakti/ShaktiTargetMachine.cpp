//===-- ShaktiTargetMachine.cpp - Define TargetMachine for Shakti -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "ShaktiTargetMachine.h"
#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "Shakti.h"
#include "ShaktiTargetObjectFile.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

extern "C" void LLVMInitializeShaktiTarget() {
  // Register the target.
  RegisterTargetMachine<ShaktiTargetMachine> X(TheShaktiTarget);
}

namespace {
/// Shakti Code Generator Pass Configuration Options.
class ShaktiPassConfig : public TargetPassConfig {
public:
  ShaktiPassConfig(ShaktiTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  ShaktiTargetMachine &getShaktiTargetMachine() const {
    return getTM<ShaktiTargetMachine>();
  }

  bool addInstSelector() override;
};

Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                    Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

} // namespace

ShaktiTargetMachine::ShaktiTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       CodeModel::Model CM,
                                       CodeGenOpt::Level OL)
    : LLVMTargetMachine(T, "e-m:e-p:32:32", TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM), CM, OL),
      TLOF(make_unique<ShaktiTargetObjectFile>()),
      Subtarget(TT, CPU, FS, *this) {
  initAsmInfo();
}

TargetPassConfig *ShaktiTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new ShaktiPassConfig(*this, PM);
}

bool ShaktiPassConfig::addInstSelector() {
  addPass(createShaktiISelDag(getShaktiTargetMachine()));
  return false;
}
