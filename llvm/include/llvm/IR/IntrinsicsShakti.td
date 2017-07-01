//===- IntrinsicsShakti.td - Defines Shakti intrinsics ---------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the Shakti-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//

def v16i32_ptr_ty : LLVMPointerType<LLVMType<v16i32>>;

let TargetPrefix = "shakti" in {  // All intrinsics start with "llvm.shakti.".

def int_shakti_read_control_reg : Intrinsic<[llvm_i32_ty], [llvm_i32_ty], [],
	"llvm.shakti.__builtin_shakti_read_control_reg">;
def int_shakti_write_control_reg : Intrinsic<[], [llvm_i32_ty, llvm_i32_ty], [],
	"llvm.shakti.__builtin_shakti_write_control_reg">;

// Memory operations
def int_shakti_gather_loadi : Intrinsic<[llvm_v16i32_ty], [llvm_v16i32_ty],
	[IntrReadMem], "llvm.shakti.__builtin_shakti_gather_loadi">;

def int_shakti_gather_loadf : Intrinsic<[llvm_v16f32_ty], [llvm_v16i32_ty],
	[IntrReadMem], "llvm.shakti.__builtin_shakti_gather_loadf">;

def int_shakti_gather_loadi_masked : Intrinsic<[llvm_v16i32_ty], [llvm_v16i32_ty, llvm_v16i1_ty],
	[IntrReadMem], "llvm.shakti.__builtin_shakti_gather_loadi_masked">;

def int_shakti_gather_loadf_masked : Intrinsic<[llvm_v16f32_ty], [llvm_v16i32_ty, llvm_v16i1_ty],
	[IntrReadMem], "llvm.shakti.__builtin_shakti_gather_loadf_masked">;

def int_shakti_scatter_storei : Intrinsic<[], [llvm_v16i32_ty, llvm_v16i32_ty],
	[IntrWriteMem], "llvm.shakti.__builtin_shakti_scatter_storei">;

def int_shakti_scatter_storef : Intrinsic<[], [llvm_v16i32_ty, llvm_v16f32_ty],
	[IntrWriteMem], "llvm.shakti.__builtin_shakti_scatter_storef">;

def int_shakti_scatter_storei_masked : Intrinsic<[], [llvm_v16i32_ty, llvm_v16i32_ty, llvm_v16i1_ty],
	[IntrWriteMem], "llvm.shakti.__builtin_shakti_scatter_storei_masked">;

def int_shakti_scatter_storef_masked : Intrinsic<[], [llvm_v16i32_ty, llvm_v16f32_ty, llvm_v16i1_ty],
	[IntrWriteMem], "llvm.shakti.__builtin_shakti_scatter_storef_masked">;

def int_shakti_block_storei_masked : Intrinsic<[], [v16i32_ptr_ty, llvm_v16i32_ty, llvm_v16i1_ty],
	[IntrWriteMem, IntrArgMemOnly],   "llvm.shakti.__builtin_shakti_block_storei_masked">;

def int_shakti_block_storef_masked : Intrinsic<[], [v16i32_ptr_ty, llvm_v16f32_ty, llvm_v16i1_ty],
	[IntrWriteMem, IntrArgMemOnly], "llvm.shakti.__builtin_shakti_block_storef_masked">;

// Shuffle vector elements
def int_shakti_shufflei : Intrinsic<[llvm_v16i32_ty], [llvm_v16i32_ty, llvm_v16i32_ty],
	[IntrNoMem], "llvm.shakti.__builtin_shakti_shufflei">;
def int_shakti_shufflef : Intrinsic<[llvm_v16f32_ty], [llvm_v16f32_ty, llvm_v16i32_ty],
	[IntrNoMem], "llvm.shakti.__builtin_shakti_shufflef">;

}

