add_llvm_library(LLVMShaktiDesc
  ShaktiMCTargetDesc.cpp
  ShaktiMCAsmInfo.cpp
  ShaktiMCCodeEmitter.cpp
  ShaktiAsmBackend.cpp
  ShaktiELFObjectWriter.cpp
  ShaktiMCExpr.cpp
  )
add_dependencies(LLVMShaktiDesc ShaktiCommonTableGen)
