//===-- ShaktiBaseInfo.h - Top level definitions for Shakti MC ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the Shakti target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_MCTARGETDESC_SHAKTIBASEINFO_H
#define LLVM_LIB_TARGET_SHAKTI_MCTARGETDESC_SHAKTIBASEINFO_H

namespace llvm {

namespace Shakti {
// Target Operand Flag enum.
enum TOF {
  //===------------------------------------------------------------------===//
  MO_NO_FLAG,

  // MO_ABS_HI/LO - Represents the hi or low part of an absolute symbol
  // address.
  MO_ABS_HI,
  MO_ABS_LO,
};
} // namespace Shakti
} // namespace llvm
#endif // LLVM_LIB_TARGET_SHAKTI_MCTARGETDESC_SHAKTIBASEINFO_H
