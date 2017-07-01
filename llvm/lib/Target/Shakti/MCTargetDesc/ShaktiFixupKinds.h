//===-- MipsFixupKinds.h - Mips Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_SHAKTIFIXUPKINDS_H
#define LLVM_LIB_TARGET_SHAKTI_SHAKTIFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace Shakti {
// Although most of the current fixup types reflect a unique relocation
// one can have multiple fixup types for a given relocation and thus need
// to be uniquely named.
//
// This table *must* be in the save order of
// MCFixupKindInfo Infos[Shakti::NumTargetFixupKinds]
// in ShaktiAsmBackend.cpp.
//
enum Fixups {
  fixup_Shakti_Branch20 = FirstTargetFixupKind,
  fixup_Shakti_Branch25,
  fixup_Shakti_HI19,                         // Fix up 19 bit movehi value
  fixup_Shakti_IMM_LO13,                     // Immediate instruction w/ 13 bit offs

  // Marker
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};
} // namespace Shakti
} // namespace llvm

#endif // LLVM_MIPS_SHAKTIFIXUPKINDS_H
