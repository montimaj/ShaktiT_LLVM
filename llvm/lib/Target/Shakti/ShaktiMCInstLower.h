//===-- ShaktiMCInstLower.h - Lower MachineInstr to MCInst ----*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===--------------------------------------------------------------------===//
//
// ShaktiMCInstLower - Helper class used by ShaktiAsmPrinter to convert
// MachineInstrs into MCInsts
//
//===--------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_SHAKTI_SHAKTIMCINSTLOWER_H
#define LLVM_LIB_TARGET_SHAKTI_SHAKTIMCINSTLOWER_H

#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/Support/Compiler.h"

namespace llvm {
class MCContext;
class MCInst;
class MCOperand;
class MachineInstr;
class ShaktiAsmPrinter;

class LLVM_LIBRARY_VISIBILITY ShaktiMCInstLower {
public:
  explicit ShaktiMCInstLower(ShaktiAsmPrinter &asmprinter);
  void Initialize(MCContext *C);
  void Lower(const MachineInstr *MI, MCInst &OutMI) const;

private:
  MCOperand LowerOperand(const MachineOperand &MO) const;
  MCOperand LowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym) const;

  MCContext *Ctx = nullptr;
  ShaktiAsmPrinter &AsmPrinter;
};

} // namespace llvm

#endif
