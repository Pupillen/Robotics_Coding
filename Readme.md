# 针对Robotics Coding指南
本仓库收集汇总一些个人觉得常用的关于ROS等相关程序debug,profiling,加速调优等的相关工具链.
# Debug
## CPU程序Debug
## 关于逐行运行代码debug
推荐vscode ros插件,以debug模式编译节点,之后详见
- [ros节点debug,roslaunch运行debug](debug-support.md)
## Core Dump分析
- [段错误or核心转储or Segmentation Fault](Coredump.md)
## CUDA程序Debug
仅仅set(CMAKE_BUILD_TYPE Debug)不够，注意设置-g -G的flag为host与device的调试信息如下，同时安装vscode插件[Nsight Visual Studio Code Edition](https://github.com/NVIDIA/nsight-vscode-edition.git)。支持一个warp为单位逐行bug.
![thread id注意设置为32的整数倍数](Figures/notes/cudadebug.png)
- [补充参见视频](https://developer.nvidia.com/nsight-visual-studio-code-edition)
- 墙裂推荐nsight-syms组件调优profiling,[视频](https://www.youtube.com/watch?v=kKANP0kL_hk&ab_channel=POPHPC)

```cpp
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_ARCHITECTURES 52;70;75;86)

project(hellocuda LANGUAGES CXX CUDA)

add_executable(main main.cu)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(main PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-G -g>)
endif()
target_include_directories(main PUBLIC ../../include)
```


# HPC(高性能计算)
- 并行加速库推荐OpenMP,在ros中使用个人觉得方便简单,例如典型cpu-bound的进程中for循环加上适当profiling可加速10倍
- 墙裂推荐[并行编程与优化](https://github.com/parallel101/course)
# Profiling (CPU+GPU)
- [程序调优](Profiling.md)


# 编译,cmake相关系列
- 参见Doc/cmake项目管理.pdf
- 参见Doc/cmakeadvanced.pdf
- 类似墙裂推荐[并行编程与优化](https://github.com/parallel101/course)cmake相关系列
- 编译期sanity-check
```make
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#!!!!!!!!!!sanity check, compiler flags!!!!!!!!!!
add_compile_options(
  -Wall
  -Wextra
  -Weffc++
  -Werror=uninitialized
  -Werror=return-type
  -Wconversion
  -Werror=unused-result
  -Werror=suggest-override
  -Wzero-as-null-pointer-constant
  -Wmissing-declarations
  -Wold-style-cast
  -Wnon-virtual-dtor
  )
  #!!!!!!!!!!sanity check, com```cpppiler flags!!!!!!!!!!
```

