//===-- ShaktiMCAsmInfo.h - Shakti asm properties ----------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the ShaktiMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_SHAKTIMCASMINFO_H
#define LLVM_LIB_TARGET_SHAKTI_SHAKTIMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class ShaktiMCAsmInfo : public MCAsmInfo {
  virtual void anchor();

public:
  explicit ShaktiMCAsmInfo(const Triple &TT);
};

} // namespace llvm

#endif
