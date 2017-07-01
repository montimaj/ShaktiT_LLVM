set(LLVM_TARGET_DEFINITIONS Shakti.td)

tablegen(LLVM ShaktiGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM ShaktiGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM ShaktiGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM ShaktiGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM ShaktiGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM ShaktiGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM ShaktiGenCallingConv.inc -gen-callingconv)
tablegen(LLVM ShaktiGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM ShaktiGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM ShaktiGenDisassemblerTables.inc -gen-disassembler)
add_public_tablegen_target(ShaktiCommonTableGen)

add_llvm_target(ShaktiCodeGen
  ShaktiAsmPrinter.cpp
  ShaktiInstrInfo.cpp
  ShaktiISelDAGToDAG.cpp
  ShaktiISelLowering.cpp
  ShaktiFrameLowering.cpp
  ShaktiMachineFunctionInfo.cpp
  ShaktiRegisterInfo.cpp
  ShaktiSubtarget.cpp
  ShaktiTargetMachine.cpp
  ShaktiMCInstLower.cpp
  ShaktiTargetObjectFile.cpp
  )
  
add_dependencies(LLVMShaktiCodeGen intrinsics_gen)

add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
add_subdirectory(AsmParser)
add_subdirectory(InstPrinter)
add_subdirectory(Disassembler)

