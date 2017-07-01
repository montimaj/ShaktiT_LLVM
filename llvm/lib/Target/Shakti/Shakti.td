//===-- Shakti.td - Describe the Shakti Target Machine -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "ShaktiRegisterInfo.td"
include "ShaktiCallingConv.td"
include "ShaktiSchedule.td"
include "ShaktiInstrFormats.td"
include "ShaktiInstrInfo.td"

def ShaktiInstrInfo : InstrInfo;

def ShaktiAsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 1;
}

def ShaktiAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  bit isMCAsmWriter = 1;
}

def : Processor<"shakti", ShaktiItineraries, []>;

def Shakti : Target {
  let InstructionSet = ShaktiInstrInfo;
  let AssemblyWriters = [ShaktiAsmWriter];
  let AssemblyParsers = [ShaktiAsmParser];
}
