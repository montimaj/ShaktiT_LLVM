//===-- ShaktiMCTargetDesc.cpp - Shakti Target Descriptions  ---------------===//
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

#include "ShaktiMCTargetDesc.h"
#include "InstPrinter/ShaktiInstPrinter.h"
#include "ShaktiMCAsmInfo.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "ShaktiGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "ShaktiGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "ShaktiGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createShaktiMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitShaktiMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createShaktiMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitShaktiMCRegisterInfo(X, Shakti::RA_REG);
  return X;
}

static MCSubtargetInfo *
createShaktiMCSubtargetInfo(const Triple &TT, StringRef CPU, StringRef FS) {
  return createShaktiMCSubtargetInfoImpl(TT, CPU, FS);
}

static MCAsmInfo *createShaktiMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT) {
  MCAsmInfo *MAI = new ShaktiMCAsmInfo(TT);

  // Put an instruction into the common information entry (CIE), shared
  // by all frame description entries (FDE), which indicates the stack
  // pointer register (r30) is used to find the canonical frame address (CFA).
  unsigned SP = MRI.getDwarfRegNum(Shakti::SP_REG, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCStreamer *createShaktiMCStreamer(const Triple &T, MCContext &Context,
                                         MCAsmBackend &MAB,
                                         raw_pwrite_stream &OS,
                                         MCCodeEmitter *Emitter,
                                         bool RelaxAll) {
  return createELFStreamer(Context, MAB, OS, Emitter, RelaxAll);
}

static MCInstPrinter *createShaktiMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new ShaktiInstPrinter(MAI, MII, MRI);
}

namespace {

class ShaktiMCInstrAnalysis : public MCInstrAnalysis {
public:
  explicit ShaktiMCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {

    unsigned NumOps = Inst.getNumOperands();
    if (NumOps == 0)
      return false;
    switch (Info->get(Inst.getOpcode()).OpInfo[NumOps - 1].OperandType) {
    case MCOI::OPERAND_UNKNOWN:
    case MCOI::OPERAND_IMMEDIATE:
      Target = Addr + Inst.getOperand(NumOps - 1).getImm();
      return true;
    default:
      return false;
    }
  }
};

} // namespace

static MCInstrAnalysis *createShaktiMCInstrAnalysis(const MCInstrInfo *Info) {
  return new ShaktiMCInstrAnalysis(Info);
}

extern "C" void LLVMInitializeShaktiTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn A(TheShaktiTarget, createShaktiMCAsmInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheShaktiTarget, createShaktiMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheShaktiTarget, createShaktiMCRegisterInfo);

  TargetRegistry::RegisterMCCodeEmitter(TheShaktiTarget,
                                        createShaktiMCCodeEmitter);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheShaktiTarget,
                                          createShaktiMCSubtargetInfo);

  // Register the MC instruction analyzer.
  TargetRegistry::RegisterMCInstrAnalysis(TheShaktiTarget,
                                          createShaktiMCInstrAnalysis);

  // Register the ASM Backend
  TargetRegistry::RegisterMCAsmBackend(TheShaktiTarget, createShaktiAsmBackend);

  // Register the object streamer
  TargetRegistry::RegisterELFStreamer(TheShaktiTarget, createShaktiMCStreamer);

  // MC instruction printer
  TargetRegistry::RegisterMCInstPrinter(TheShaktiTarget,
                                        createShaktiMCInstPrinter);
}
