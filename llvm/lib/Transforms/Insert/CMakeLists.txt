add_llvm_loadable_module( LLVMInsertInst
  InsertInst.cpp

  DEPENDS
  intrinsics_gen
  PLUGIN_TOOL
  opt
  )
