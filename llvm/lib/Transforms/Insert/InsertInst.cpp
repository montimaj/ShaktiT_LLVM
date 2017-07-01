/**
 * Intrinsic testing.
 * @author Sayantan Majumdar
*/
 
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Function.h"
#include "llvm/Pass.h"
#include "llvm/IR/Constants.h"
#include "llvm/Support/raw_ostream.h"
 
#include<vector>
 
using namespace llvm;
 
#define DEBUG_TYPE "insertinst"
 
namespace {
  struct InsertInstruction : public FunctionPass {
    static char ID; // Pass identification, replacement for typeid
    InsertInstruction() : FunctionPass(ID) {}
  private:
    void eraseOldInstructions(std::vector<Instruction*>& v) {
        for(auto I: v) I->eraseFromParent();
    }
   
    bool runOnFunction(Function &F) override {
        bool modified = false;
        std::vector<Instruction*> old_inst_vec;
        for(auto& B: F) {
            for(auto& I: B) {
                if(I.getOpcode() == Instruction::FAdd) {
                    old_inst_vec.push_back(&I);
                    Value* lhs = I.getOperand(0);
                    Value* rhs = I.getOperand(1);            
                    IRBuilder<> builder(&I);
                    Value* add_value = builder.CreateFAdd(lhs, rhs);
                    Value* five =  ConstantFP::get(lhs->getType(),5.0);                    
                    Value* final_value = builder.CreateFAdd(add_value, five);
                    //Type* ty[] = {Type::getFloatTy(F.getParent()->getContext())};
                    /*Function* new_func = Intrinsic::getDeclaration(F.getParent(), Intrinsic::shakti_block_storef_masked);
                    Value* sqrt = CallInst::Create(new_func, final_value, "", &I);*/
                    I.replaceAllUsesWith(final_value);
                    modified = true;
                }
            }
        }
        if(modified) eraseOldInstructions(old_inst_vec);
        return modified;
    }
  };
}
 
char InsertInstruction::ID = 0;
static RegisterPass<InsertInstruction> X("insertinst", "Insert Instruction");
