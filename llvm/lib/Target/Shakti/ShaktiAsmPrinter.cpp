//===-- ShaktiAsmPrinter.cpp - Shakti LLVM assembly writer -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "shakti-asm-printer"

#include "ShaktiAsmPrinter.h"
#include "InstPrinter/ShaktiInstPrinter.h"
#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "Shakti.h"
#include "ShaktiInstrInfo.h"
#include "ShaktiMCInstLower.h"
#include "ShaktiTargetMachine.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetLoweringObjectFile.h"

using namespace llvm;

void ShaktiAsmPrinter::EmitInstruction(const MachineInstr *MI) {
  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();

  do {
    MCInst TmpInst;
    MCInstLowering.Lower(MI, TmpInst);
    EmitToStreamer(*OutStreamer, TmpInst);
  } while ((++I != E) && I->isInsideBundle());
}

void ShaktiAsmPrinter::EmitFunctionBodyStart() {
  MCInstLowering.Initialize(&MF->getContext());
}

// Print operand for inline assembly
bool ShaktiAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                      unsigned AsmVariant,
                                      const char *ExtraCode, raw_ostream &O) {

  if (ExtraCode && ExtraCode[0])
    return AsmPrinter::PrintAsmOperand(MI, OpNo, AsmVariant, ExtraCode, O);

  const MachineOperand &MO = MI->getOperand(OpNo);
  if (MO.getType() == MachineOperand::MO_Register) {
    O << ShaktiInstPrinter::getRegisterName(MO.getReg());
    return false;
  } else if (MO.getType() == MachineOperand::MO_Immediate) {
    O << MO.getImm();
    return false;
  }

  return true;
}

bool ShaktiAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                            unsigned OpNum, unsigned AsmVariant,
                                            const char *ExtraCode,
                                            raw_ostream &O) {

  const MachineOperand &MO = MI->getOperand(OpNum);
  assert(MO.isReg() && "unexpected inline asm memory operand");
  O << "(" << ShaktiInstPrinter::getRegisterName(MO.getReg()) << ")";

  return false;
}

// Force static initialization.
extern "C" void LLVMInitializeShaktiAsmPrinter() {
  RegisterAsmPrinter<ShaktiAsmPrinter> X(TheShaktiTarget);
}
