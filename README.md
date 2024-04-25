新模式说明
==============================

新模式包括新曲线划分模式和新加权混合模式，要开启或关闭曲线划分模式，请修改[TypeDef.h](source\Lib\CommonLib\TypeDef.h)中的```GPM_CURVE```。同样新加权混合模式也可以通过修改```GPM_BLEND```来实现。

所有相关修改都用宏```GPM_CURVE```和宏```GPM_BLEND```包括在内。当两个宏都被置0的时为原版ECM。

新模式主要通过修改[Rom.cpp](source\Lib\CommonLib\Rom.cpp)中的GPM模式的加权融合模板，修改的结果会输出为```g_geoWeights.txt```，是一个112x112的16bit二进制矩阵，表示各像素的权重。

要进行通测，请按照以下格式配置命令行参数

```bash
EncoderApp -c path/to/cfg/encoder_randomaccess_ecm.cfg -i path/to/test.yuv
-o path/to/reconstruction.yuv -b path/to/output.bitstream -wdt 1920 -hgt 1080 -f 500 -fr 30
```

命令行各参数的具体含义可通过查询[software-manual](doc\software-manual.pdf)或直接运行```EncoderApp --help```来查看

下面是是原ECM自带的一些说明文档，简要介绍了ECM，以及如何编译ECM。

ECM reference software
==============================

This software package is the reference software for Enhanced Compression Model (ECM). The reference software includes both encoder and decoder functionality.

Reference software is useful in aiding users of a video coding standard to establish and test conformance and interoperability, and to educate users and demonstrate the capabilities of the test model.

The software has been jointly developed by the ITU-T Video Coding Experts Group (VCEG, Question 6 of ITU-T Study Group 16) and the ISO/IEC Moving Picture Experts Group (MPEG, Working Group 11 of Subcommittee 29 of ISO/IEC Joint Technical Committee 1).

A software manual, which contains usage instructions, can be found in the "doc" subdirectory of this software package.

Build instructions
==================

The CMake tool is used to create platform-specific build files.

Although CMake may be able to generate 32-bit binaries, **it is generally suggested to build 64-bit binaries**. 32-bit binaries are not able to access more than 2GB of RAM, which will not be sufficient for coding larger image formats. Building in 32-bit environments is not tested and will not be supported.

Build instructions for plain CMake (suggested)
----------------------------------------------

**Note:** A working CMake installation is required for building the software.

CMake generates configuration files for the compiler environment/development environment on each platform.
The following is a list of examples for Windows (MS Visual Studio), macOS (Xcode) and Linux (make).

Open a command prompt on your system and change into the root directory of this project.

Create a build directory in the root directory:

```bash
mkdir build 
```

Use one of the following CMake commands, based on your platform. Feel free to change the commands to satisfy
your needs.

**Windows Visual Studio 2015/17/19 64 Bit:**

Use the proper generator string for generating Visual Studio files, e.g. for VS 2015:

```bash
cd build
cmake .. -G "Visual Studio 14 2015 Win64"
```

Then open the generated solution file in MS Visual Studio.

For VS 2017 use "Visual Studio 15 2017 Win64", for VS 2019 use "Visual Studio 16 2019".

Visual Studio 2019 also allows you to open the CMake directory directly. Choose "File->Open->CMake" for this option.

**macOS Xcode:**

For generating an Xcode workspace type:

```bash
cd build
cmake .. -G "Xcode"
```

Then open the generated work space in Xcode.

For generating Makefiles with optional non-default compilers, use the following commands:

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9
```

In this example the brew installed GCC 9 is used for a release build.

**Linux**

For generating Linux Release Makefile:

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

For generating Linux Debug Makefile:

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

Then type

```bash
make -j
```

For more details, refer to the CMake documentation: <https://cmake.org/cmake/help/latest/>

Build instructions for make
---------------------------

**Note:** The build instructions in this section require the make tool and Python to be installed, which are
part of usual Linux and macOS environments. See below for installation instruction for Python and GnuWin32
on Windows.

Open a command prompt on your system and change into the root directory of this project.

To use the default system compiler simply call:

```bash
make all
```

**MSYS2 and MinGW (Windows)**

**Note:** Build files for MSYS MinGW were added on request. The build platform is not regularily tested and can't be supported.

Open an MSYS MinGW 64-Bit terminal and change into the root directory of this project.

Call:

```bash
make all toolset=gcc
```

The following tools need to be installed for MSYS2 and MinGW:

Download CMake: <http://www.cmake.org/> and install it.

Python and GnuWin32 are not mandatory, but they simplify the build process for the user.

python:    <https://www.python.org/downloads/release/python-371/>

gnuwin32:  <https://sourceforge.net/projects/getgnuwin32/files/getgnuwin32/0.6.30/GetGnuWin32-0.6.3.exe/download>

To use MinGW, install MSYS2: <http://repo.msys2.org/distrib/msys2-x86_64-latest.exe>

Installation instructions: <https://www.msys2.org/>

Install the needed toolchains:

```bash
pacman -S --needed base-devel mingw-w64-i686-toolchain mingw-w64-x86_64-toolchain git subversion mingw-w64-i686-cmake mingw-w64-x86_64-cmake
```
