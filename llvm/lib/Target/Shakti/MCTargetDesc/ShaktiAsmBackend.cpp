//===-- ShaktiASMBackend.cpp - Shakti Asm Backend  -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the ShaktiAsmBackend class.
//
//===----------------------------------------------------------------------===//
//

#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "ShaktiFixupKinds.h"
#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {
class ShaktiAsmBackend : public MCAsmBackend {
  Triple::OSType OSType;

public:
  ShaktiAsmBackend(const Target &T, Triple::OSType _OSType)
      : MCAsmBackend(), OSType(_OSType) {}

  MCObjectWriter *createObjectWriter(raw_pwrite_stream &OS) const override {
    return createShaktiELFObjectWriter(
        OS, MCELFObjectTargetWriter::getOSABI(OSType));
  }

  unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                            MCContext *Ctx) const {
    const MCFixupKindInfo &Info = getFixupKindInfo(Fixup.getKind());

    switch ((unsigned int)Fixup.getKind()) {
    case Shakti::fixup_Shakti_Branch20:
    case Shakti::fixup_Shakti_Branch25: {
      // Divide by 4, because the hardware will multiply it.
      Value = static_cast<uint64_t>(static_cast<int64_t>(Value) / 4);
      if (!APInt(64, Value).isSignedIntN(Info.TargetSize)) {
        Ctx->reportError(Fixup.getLoc(), "fixup out of range");
        return 0;
      }

      return Value;
    }
    case Shakti::fixup_Shakti_HI19:
      // Immediate value is split between two instruction fields. Align
      // to proper locations
      return ((Value >> 13) & 0x1f) | (((Value >> 18) & 0x3fff) << 10);
    default:;
    }

    return Value;
  }

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsPCRel) const override {

    const MCFixupKindInfo &Info = getFixupKindInfo(Fixup.getKind());
    Value = adjustFixupValue(Fixup, Value, &Asm.getContext());
    unsigned Offset = Fixup.getOffset();
    unsigned NumBytes = 4;

    uint64_t CurVal = 0;
    for (unsigned i = 0; i != NumBytes; ++i) {
      CurVal |= (uint64_t)((uint8_t)Data[Offset + i]) << (i * 8);
    }

    uint64_t Mask = ((uint64_t)(-1) >> (64 - Info.TargetSize));

    Value <<= Info.TargetOffset;
    Mask <<= Info.TargetOffset;
    CurVal |= Value & Mask;

    // Write out the fixed up bytes back to the code/data bits.
    for (unsigned i = 0; i != NumBytes; ++i) {
      Data[Offset + i] = (uint8_t)((CurVal >> (i * 8)) & 0xff);
    }
  }

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
    const static MCFixupKindInfo Infos[Shakti::NumTargetFixupKinds] = {
        // This table *must* be in same the order of fixup_* kinds in
        // ShaktiFixupKinds.h.
        //
        // name                          offset  bits  flags
        {"fixup_Shakti_Branch20", 5, 20, MCFixupKindInfo::FKF_IsPCRel},
        {"fixup_Shakti_Branch25", 0, 25, MCFixupKindInfo::FKF_IsPCRel},
        {"fixup_Shakti_HI19", 0, 32, 0},
        {"fixup_Shakti_IMM_LO13", 10, 13, 0},  // This is unsigned, don't take top bit
    };

    if (Kind < FirstTargetFixupKind)
      return MCAsmBackend::getFixupKindInfo(Kind);

    assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
           "Invalid kind!");
    return Infos[Kind - FirstTargetFixupKind];
  }

  bool mayNeedRelaxation(const MCInst &Inst) const override { return false; }

  /// fixupNeedsRelaxation - Target specific predicate for whether a given
  /// fixup requires the associated instruction to be relaxed.
  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *DF,
                            const MCAsmLayout &Layout) const override {
    return false;
  }

  void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                        MCInst &Res) const override {
    assert(0 && "relaxInstruction() unimplemented");
  }

  // This gets called to align. If strings are emitted in the text area,
  // this may not be a multiple of the instruction size. Just fill with
  // zeroes.
  bool writeNopData(uint64_t Count, MCObjectWriter *OW) const override {
    OW->WriteZeros(Count);
    return true;
  }

private:
  unsigned getNumFixupKinds() const override {
    return Shakti::NumTargetFixupKinds;
  }

}; // class ShaktiAsmBackend

} // namespace

// MCAsmBackend
MCAsmBackend *llvm::createShaktiAsmBackend(const Target &T,
                                          const MCRegisterInfo &MRI,
                                          const Triple &TT, StringRef CPU,
                                          const MCTargetOptions &Options) {
  return new ShaktiAsmBackend(T, Triple(TT).getOS());
}
