//===-- ShaktiMCTargetDesc.h - Shakti Target Descriptions ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Shakti specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_SHAKTIMCTARGETDESC_H
#define LLVM_LIB_TARGET_SHAKTI_SHAKTIMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {
class Target;
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class raw_ostream;
class StringRef;
class Triple;

extern Target TheShaktiTarget;

MCCodeEmitter *createShaktiMCCodeEmitter(const MCInstrInfo &MCII,
                                        const MCRegisterInfo &MRI,
                                        MCContext &Ctx);

MCObjectWriter *createShaktiELFObjectWriter(raw_pwrite_stream &OS,
                                           uint8_t OSABI);

MCAsmBackend *createShaktiAsmBackend(const Target &T, const MCRegisterInfo &MRI,
                                    const Triple &TT, StringRef CPU,
                                    const MCTargetOptions &Options);

} // namespace llvm

// Defines symbolic names for Shakti registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "ShaktiGenRegisterInfo.inc"

// Defines symbolic names for the Shakti instructions.
//
#define GET_INSTRINFO_ENUM
#include "ShaktiGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "ShaktiGenSubtargetInfo.inc"

#endif
