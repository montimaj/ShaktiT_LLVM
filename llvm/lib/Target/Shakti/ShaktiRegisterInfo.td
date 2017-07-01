//===-- ShaktiRegisterInfo.td - Shakti Register defs ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Shakti register file
//===----------------------------------------------------------------------===//

let Namespace = "Shakti" in {
  class ShaktiReg<string n, bits<16> number> : Register<n> {
    field bits<16> HWEncoding = number;
  }

  foreach i = 0-28 in {
    def S#i : ShaktiReg<"s"#i, i>, DwarfRegNum<[i]>;
  }

  def FP_REG : ShaktiReg<"fp", 29>, DwarfRegNum<[29]>;
  def SP_REG : ShaktiReg<"sp", 30>, DwarfRegNum<[30]>;
  def RA_REG : ShaktiReg<"ra", 31>, DwarfRegNum<[31]>;

  foreach i = 0-31 in {
    def V#i : ShaktiReg<"v"#i, i>, DwarfRegNum<[!add(i, 32)]>;
  }
}

def GPR32 : RegisterClass<"Shakti", [i32, f32, v16i1], 32,
  (add (sequence "S%u", 0, 28), FP_REG, SP_REG, RA_REG)>;

def VR512 : RegisterClass<"Shakti", [v16i32, v16f32], 512, (sequence "V%u", 0, 31)>;

