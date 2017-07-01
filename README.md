This is a toolchain for the T-class SHAKTI processor
[Shakti](http://rise.cse.iitm.ac.in/shakti.html), based on
[LLVM](http://llvm.org/).  Currently only the framework is ready. Lots of changes are to be made. It includes a C/C++ compiler (clang), assembler,
linker and debugger (lldb).

While this project includes a C/C++ compiler, the LLVM backend can support
any language.

# Building

## Required Software

The following sections describe how to install these packages.

- gcc 4.8+ or Apple clang 4.2+
- cmake 3.4.3+
- python 2.7
- libxml (including headers)
- zlib (including headers)
- bison 2.7+
- flex 2.5+
- swig 3.0.11+ (http://www.swig.org/) with python wrappers
- libedit (http://thrysoee.dk/editline/)
- ncurses

## Building on Linux

You can install required packages using the built-in package manager (apt-get,
yum, etc). As LLVM needs newer versions of many packages, you should be on
a recent version of your Linux distribution. Instructions are below are for Ubuntu
(which must be on at least version 16). You may need to change the package names
for other distributions:

    sudo apt-get install libxml2-dev cmake gcc g++ python bison flex \
        zlib1g-dev swig python-dev libedit-dev libncurses5-dev

    $ git clone https://github.com/montimaj/ShaktiT_LLVM
    $ cd ShaktiT_LLVM
    $ mkdir build && cd build
    $ cmake -G Ninja ../llvm
    $ ninja
