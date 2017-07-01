//===-- ShaktiELFObjectWriter.cpp - Shakti ELF Writer ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/ShaktiFixupKinds.h"
#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {
class ShaktiELFObjectWriter : public MCELFObjectTargetWriter {
public:
  explicit ShaktiELFObjectWriter(uint8_t OSABI);

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
} // namespace

ShaktiELFObjectWriter::ShaktiELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(/*Is64Bit*/ false, OSABI, ELF::EM_SHAKTI,
                              /*HasRelocationAddend*/ true) {}

unsigned ShaktiELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  unsigned Type;
  unsigned Kind = (unsigned)Fixup.getKind();
  switch (Kind) {
  default:
    llvm_unreachable("Invalid fixup kind!");
  case FK_Data_4:
    Type = ELF::R_SHAKTI_ABS32;
    break;

  case Shakti::fixup_Shakti_Branch20:
    assert(IsPCRel);
    Type = ELF::R_SHAKTI_BRANCH20;
    break;

  case Shakti::fixup_Shakti_Branch25:
    assert(IsPCRel);
    Type = ELF::R_SHAKTI_BRANCH25;
    break;

  case Shakti::fixup_Shakti_HI19:
    assert(!IsPCRel);
    Type = ELF::R_SHAKTI_HI19;
    break;

  case Shakti::fixup_Shakti_IMM_LO13:
    assert(!IsPCRel);
    Type = ELF::R_SHAKTI_IMM_LO13;
    break;
  }
  return Type;
}

MCObjectWriter *llvm::createShaktiELFObjectWriter(raw_pwrite_stream &OS,
                                                 uint8_t OSABI) {
  MCELFObjectTargetWriter *MOTW = new ShaktiELFObjectWriter(OSABI);
  return createELFObjectWriter(MOTW, OS, true);
}
