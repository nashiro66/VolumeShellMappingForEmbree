## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

To compile tutorials contained in this folder first create and enter a
build folder inside the tutorial you want to compile (e.g. minimal
tutorial):

    cd minimal
    mkdir build
    cd build

Then configure the tutorial you would like to compile (e.g. minimal),
by pointing to the Embree and TBB installation on your system. Under
Linux execute:

    cmake -D CMAKE_BUILD_TYPE=Release -D embree_DIR=`pwd`/../../../lib/cmake/embree-4.0.4/ -D TBB_DIR=path_to_tbb_install ..

And under Windows:

    cmake -G Ninja
          -D CMAKE_CXX_COMPILER=clang++
          -D CMAKE_C_COMPILER=clang
          -D CMAKE_CXX_FLAGS=-fuse-ld=link
          -D CMAKE_C_FLAGS=-fuse-ld=link
          -D CMAKE_BUILD_TYPE=Release
          -D embree_DIR=%cd%\..\..\..\lib\cmake\embree-4.0.4\
          -D TBB_DIR=path_to_tbb\lib\cmake\tbb ..

Now you can build the tutorial:

    cmake --build .

In order to run the tutorial Under Windows you have to set the PATH to
the bin folder in order to find the DLLs:

  set PATH=%cd%\..\..\..\bin;%PATH%

Now you can run the tutorial:

    ./minimal
    ./minimal_sycl
