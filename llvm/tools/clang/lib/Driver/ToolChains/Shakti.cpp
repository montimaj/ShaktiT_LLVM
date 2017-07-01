//===--- Shakti.cpp - Shakti ToolChain Implementations ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Shakti.h"
#include "Arch/ARM.h"
#include "Arch/Mips.h"
#include "Arch/Sparc.h"
#include "CommonArgs.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Options.h"
#include "llvm/Option/ArgList.h"

using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;
using namespace clang;
using namespace llvm::opt;

void shakti::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                                 const InputInfo &Output,
                                 const InputInfoList &Inputs,
                                 const ArgList &Args,
                                 const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  if (Output.isFilename()) {
    CmdArgs.push_back("-o");
    CmdArgs.push_back(Output.getFilename());
  } else {
    assert(Output.isNothing() && "Invalid output.");
  }

  AddLinkerInputs(getToolChain(), Inputs, Args, CmdArgs, JA);

  const char *Exec = Args.MakeArgString(getToolChain().GetProgramPath("ld.lld"));
  C.addCommand(llvm::make_unique<Command>(JA, *this, Exec, CmdArgs, Inputs));
}

ShaktiToolChain::ShaktiToolChain(const Driver &D, const llvm::Triple &Triple,
                               const llvm::opt::ArgList &Args)
	:	ToolChain(D, Triple, Args)
{
  getProgramPaths().push_back(getDriver().getInstalledDir());
  if (getDriver().getInstalledDir() != getDriver().Dir)
    getProgramPaths().push_back(getDriver().Dir);
}

ShaktiToolChain::~ShaktiToolChain()
{
}

bool ShaktiToolChain::IsIntegratedAssemblerDefault() const
{
  return true;
}

bool ShaktiToolChain::isPICDefault() const
{
  return false;
}

bool ShaktiToolChain::isPIEDefault() const
{
  return false;
}

bool ShaktiToolChain::isPICDefaultForced() const
{
  return false;
}

void ShaktiToolChain::addClangTargetOptions(const ArgList &DriverArgs,
                                  ArgStringList &CC1Args) const {
  CC1Args.push_back("-nostdsysteminc");
  if (DriverArgs.hasFlag(options::OPT_fuse_init_array,
                         options::OPT_fno_use_init_array,
                         true))
  {
    CC1Args.push_back("-fuse-init-array");
  }
}

// Emit .eh_frame to allow stack unwinding.
bool ShaktiToolChain::IsUnwindTablesDefault() const {
  return true;
}

Tool *ShaktiToolChain::buildLinker() const {
  return new tools::shakti::Linker(*this);
}

