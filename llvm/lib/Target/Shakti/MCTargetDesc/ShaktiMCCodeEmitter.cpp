//===-- ShaktiMCCodeEmitter.cpp - Convert Shakti code to machine code  -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the ShaktiMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "mccodeemitter"

#include "MCTargetDesc/ShaktiFixupKinds.h"
#include "MCTargetDesc/ShaktiMCExpr.h"
#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");

namespace {
class ShaktiMCCodeEmitter : public MCCodeEmitter {
public:
  ShaktiMCCodeEmitter(const MCInstrInfo &mcii, MCContext &ctx) : Ctx(ctx) {}

  // getBinaryCodeForInstr - TableGen'erated function for getting the
  // binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  // getMachineOpValue - Return binary encoding of operand. If the machine
  // operand requires relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned encodeMemoryOpValue(const MCInst &MI, unsigned Op,
                               SmallVectorImpl<MCFixup> &Fixups,
                               const MCSubtargetInfo &STI) const;

  unsigned encodeBranchTargetOpValue20(const MCInst &MI, unsigned OpNo,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const;
  unsigned encodeBranchTargetOpValue25(const MCInst &MI, unsigned OpNo,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const;

  // Emit one byte through output stream (from MCBlazeMCCodeEmitter)
  void EmitByte(unsigned char C, raw_ostream &OS) const { OS << (char)C; }

  void EmitLEConstant(uint64_t Val, unsigned Size, raw_ostream &OS) const {
    assert(Size <= 8 && "size too big in emit constant");

    for (unsigned i = 0; i != Size; ++i) {
      EmitByte(Val & 255, OS);
      Val >>= 8;
    }
  }

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

private:
  ShaktiMCCodeEmitter(const ShaktiMCCodeEmitter &) = delete;
  void operator=(const ShaktiMCCodeEmitter &) = delete;
  MCContext &Ctx;
};
} // namespace

MCCodeEmitter *llvm::createShaktiMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {

  return new ShaktiMCCodeEmitter(MCII, Ctx);
}

/// getMachineOpValue - Return binary encoding of operand.
unsigned
ShaktiMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  assert(MO.isExpr());
  const MCExpr *Expr = MO.getExpr();
  if (const ShaktiMCExpr *McExpr = dyn_cast<ShaktiMCExpr>(Expr)) {
    switch (McExpr->getKind()) {
    case ShaktiMCExpr::VK_Shakti_ABS_HI:
      Fixups.push_back(MCFixup::create(0, MO.getExpr(), MCFixupKind(
                       Shakti::fixup_Shakti_HI19)));
      break;

    case ShaktiMCExpr::VK_Shakti_ABS_LO:
      Fixups.push_back(MCFixup::create(0, MO.getExpr(), MCFixupKind(
                       Shakti::fixup_Shakti_IMM_LO13)));
      break;

    default:
      llvm_unreachable("Unknown fixup type");
    }
  }

  return 0;
}

// encodeBranchTargetOpValue - Return binary encoding of the jump
// target operand. If the machine operand requires relocation,
// record the relocation and return zero.
unsigned ShaktiMCCodeEmitter::encodeBranchTargetOpValue20(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {

  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() &&
         "encodeBranchTargetOpValue expects only expressions or an immediate");

  Fixups.push_back(MCFixup::create(0, MO.getExpr(),
                                   MCFixupKind(Shakti::fixup_Shakti_Branch20),
                                   MI.getLoc()));
  return 0;
}

unsigned ShaktiMCCodeEmitter::encodeBranchTargetOpValue25(
    const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups,
    const MCSubtargetInfo &STI) const {

  const MCOperand &MO = MI.getOperand(OpNo);
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() &&
         "encodeBranchTargetOpValue expects only expressions or an immediate");

  Fixups.push_back(MCFixup::create(0, MO.getExpr(),
                                   MCFixupKind(Shakti::fixup_Shakti_Branch25),
                                   MI.getLoc()));
  return 0;
}

void ShaktiMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  // Get instruction encoding and emit it
  ++MCNumEmitted; // Keep track of the number of emitted insns.
  unsigned Value = getBinaryCodeForInstr(MI, Fixups, STI);
  EmitLEConstant(Value, 4, OS);
}

// Encode Shakti Memory Operand.  The result is a packed field with the
// register in the low 5 bits and the offset in the remainder.  The instruction
// patterns will put these into the proper part of the instruction
// (ShaktiInstrFormats.td).
unsigned
ShaktiMCCodeEmitter::encodeMemoryOpValue(const MCInst &MI, unsigned Op,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  unsigned encoding;

  MCOperand baseReg;
  MCOperand offsetOp;

  if (MI.getOpcode() == Shakti::STORE_SYNC) {
    // Store sync has an additional machine operand for the success value
    baseReg = MI.getOperand(2);
    offsetOp = MI.getOperand(3);
  } else {
    baseReg = MI.getOperand(1);
    offsetOp = MI.getOperand(2);
  }

  // Register
  // This is register/offset.
  assert(baseReg.isReg() && "First operand is not register.");
  encoding = Ctx.getRegisterInfo()->getEncodingValue(baseReg.getReg());

  assert(offsetOp.isImm() && "Second operand of memory op is unknown type.");
  encoding |= static_cast<short>(offsetOp.getImm()) << 5;

  return encoding;
}

#include "ShaktiGenMCCodeEmitter.inc"
