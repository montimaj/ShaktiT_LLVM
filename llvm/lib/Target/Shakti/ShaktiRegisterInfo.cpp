//===------- ShaktiRegisterInfo.cpp - Shakti Register Information -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Shakti implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#include "ShaktiRegisterInfo.h"
#include "MCTargetDesc/ShaktiMCTargetDesc.h"
#include "Shakti.h"
#include "ShaktiInstrInfo.h"
#include "ShaktiSubtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_REGINFO_TARGET_DESC
#include "ShaktiGenRegisterInfo.inc"

using namespace llvm;

ShaktiRegisterInfo::ShaktiRegisterInfo() : ShaktiGenRegisterInfo(Shakti::FP_REG) {}

const uint16_t *
ShaktiRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  return ShaktiCSR_SaveList;
}

const uint32_t *
ShaktiRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                        CallingConv::ID) const {
  return ShaktiCSR_RegMask;
}

BitVector ShaktiRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  Reserved.set(Shakti::SP_REG);
  Reserved.set(Shakti::RA_REG);
  Reserved.set(Shakti::FP_REG);
  return Reserved;
}

const TargetRegisterClass *
ShaktiRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const {
  return &Shakti::GPR32RegClass;
}

void ShaktiRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MBBI,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {

  assert(SPAdj == 0 && "Unexpected");

  MachineInstr &MI = *MBBI;
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  MachineFunction &MF = *MI.getParent()->getParent();
  const TargetFrameLowering &TFL = *MF.getSubtarget().getFrameLowering();
  MachineFrameInfo &MFI = MF.getFrameInfo();

  // Round stack size to multiple of 64, consistent with frame pointer info.
  int stackSize = alignTo(MFI.getStackSize(), TFL.getStackAlignment());

  // Frame index is relative to where SP is before it is decremented on
  // entry to the function.  Need to add stackSize to adjust for this.
  int64_t Offset = MF.getFrameInfo().getObjectOffset(FrameIndex) +
                   MI.getOperand(FIOperandNum + 1).getImm() + stackSize;

  // Determine where callee saved registers live in the frame
  const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  int MinCSFI = 0;
  int MaxCSFI = -1;
  if (CSI.size()) {
    MinCSFI = CSI[0].getFrameIdx();
    MaxCSFI = CSI[CSI.size() - 1].getFrameIdx();
  }

  // When we save callee saved registers (which includes FP), we must use
  // the SP reg, because FP is not set up yet.
  unsigned FrameReg;
  if (FrameIndex >= MinCSFI && FrameIndex <= MaxCSFI)
    FrameReg = Shakti::SP_REG;
  else
    FrameReg = getFrameRegister(MF);

  // Replace frame index with a frame pointer reference.
  if (isInt<14>(Offset)) {
    // If the offset is small enough to fit in the immediate field, directly
    // encode it.
    MI.getOperand(FIOperandNum).ChangeToRegister(FrameReg, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
  } else if (isInt<25>(Offset)) {
    DebugLoc DL = MBBI->getDebugLoc();
    MachineBasicBlock &MBB = *MBBI->getParent();
    const ShaktiInstrInfo &TII =
        *static_cast<const ShaktiInstrInfo *>(MF.getSubtarget().getInstrInfo());

    MachineRegisterInfo &RegInfo = MBB.getParent()->getRegInfo();
    unsigned Reg = RegInfo.createVirtualRegister(&Shakti::GPR32RegClass);
    BuildMI(MBB, MBBI, DL, TII.get(Shakti::MOVEHI)).addReg(Reg, RegState::Define)
      .addImm((Offset >> 13) & 0x7ffff);
    BuildMI(MBB, MBBI, DL, TII.get(Shakti::ADDISSS)).addReg(Reg, RegState::Define)
      .addReg(FrameReg)
      .addReg(Reg);

    MI.getOperand(FIOperandNum).ChangeToRegister(Reg, false, false, true /* isKill */);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset & 0x1fff);
  } else
    report_fatal_error("frame index out of bounds: frame too large");
}

unsigned ShaktiRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  return TFI->hasFP(MF) ? Shakti::FP_REG : Shakti::SP_REG;
}

bool ShaktiRegisterInfo::requiresRegisterScavenging(
    const MachineFunction &MF) const {
  return true;
}

bool ShaktiRegisterInfo::trackLivenessAfterRegAlloc(
    const MachineFunction &MF) const {
  return true;
}

bool ShaktiRegisterInfo::requiresFrameIndexScavenging(
    const MachineFunction &MF) const {
  return true;
}
