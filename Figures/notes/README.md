# notes
# GPU profiling (见/我的坚果云/计算机) 
- nsys profile -t nvtx,cuda  --stats=true -f true -o withouteigen terrain_analyzer
nsight-sys
# GPU cuda 编程debug
set(CMAKE_BUILD_TYPE Debug)不够，注意设置-g -G的flag为host与device的调试信息如下，同时安装插件nsight-vscode。主要thread是32为单位增加的，设置时不能随意设置如下
![thread id注意设置为32的整数倍数](cudadebug.png)
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
# Valgrind & callgrind &cachegrind & 再使用kcachegrind分析相对而言更加好用，详细参见profiling_roslaunch_prefix.md
- 参见roslaunch_Tutorials_Profiling roslaunch nodes - ROS Wiki.pdf与roslaunch_Tutorials_Roslaunch Nodes in Valgrind or GDB - ROS Wiki.pdf
## 注意输出的文件在~/.ros/下,一般情况下不制定--cachegrind-out-file或者--callgrind-out-file，方便kcachegrind导入
### kcachegrind callgrind.out.158364或者kcachegrind cachegrind.out.170407类似,GUI有时候不能找到文件，命令行打开

```cpp
    <node pkg="plan_manager" type="plan_manager" name="plan_manager" output="screen" launch-prefix="valgrind --tool=cachegrind --branch-sim=yes"> 
        <rosparam file="$(find plan_manager)/config/config.yaml" command="load" />
    </node>

    <node pkg="plan_manager" type="plan_manager" name="plan_manager" output="screen" launch-prefix="valgrind --tool=callgrind  "> 
        <rosparam file="$(find plan_manager)/config/config.yaml" command="load" />
    </node>
```
# gperftools分析内存与cpu占用，官方文档

https://gperftools.github.io/gperftools/cpuprofile.html

https://github.com/ethz-asl/programming_guidelines/wiki/Profiling-Code

个人尝试推荐使用方法3，参见图片gperftools 中 CPU Profiler 不工作问题的解法.png比方法一二靠谱
具体例如ego_planner_node:

```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <gperftools/profiler.h>
#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  ProfilerStart("/home/ztr/Pipline-Swarm-Formation/heap/ego_planner_node.prof");
  HeapProfilerStart("/home/ztr/Pipline-Swarm-Formation/heap/ego_planner_node_memory.log");
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  // ros::Duration(1.0).sleep();
  ros::spin();
   HeapProfilerStop();
  ProfilerStop();
  return 0;

//对应CMakeLists.txt多出一些行数如

target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  profiler //链接上profiler,分析CPU需要，
  tcmalloc//分析内存则需要！！！！重要，参见https://gperftools.github.io/gperftools/heapprofile.html
  )
set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed") #https://stackoverflow.com/questions/24532853/how-can-i-add-linker-flag-for-libraries-with-cmake


}
// 最后如下分析
google-pprof --pdf devel/lib/ego_planner/ego_planner_node heap/ego_planner_node.prof > test.pdf
```

# 获得所有函数调用的情况

//然后为了显示出全部函数调用的情况，需要在各个link的库中添加头文件，添加代码避免被编译器优化，具体原因的如“gperftools 中 CPU Profiler 不工作问题的解法.png“显示，操作流程为”性能分析.png“，伦哥分支当中操作。
```cpp

#include <gperftools/profiler.h>

并且代码当中添加如下

 volatile bool tmp = false;
if (tmp) ProfilerStop();

CMakeLists.txt中链接profiler如下

target_link_libraries( traj_opt

${catkin_LIBRARIES}

profiler

)

target_link_libraries( plan_env

${catkin_LIBRARIES}

${PCL_LIBRARIES}

profiler

)
等等，最后结果如test2.pdf所示。内存分析结果如test4.pdf
```


# coreDump:debug

https://zhuanlan.zhihu.com/p/459530578结合https://blog.csdn.net/weixin_33941707/article/details/112592730:使用systemd-coredump


sudo apt install systemd-coredump
修改core dump大小

## sudo vim /etc/systemd/coredump.conf 注意取消注释，ProcessSizeMax，ExternalSizeMax与JournalSizeMax

[Coredump]
#Storage=external
#Compress=yes
ProcessSizeMax=8G
ExternalSizeMax=8G
JournalSizeMax=8G
#MaxUse=
#KeepFree=

# 制定cmake特定版本找到cmakeconfig

sudo ln -sf /usr/local/include/pcl-1.12/pcl /usr/include/pcl
set(PCL_DIR "/usr/local/share/pcl-1.12/")  

## 一定注意上述目录中/不能少set(PCL_DIR "/usr/local/share/pcl-1.12")是错的，必须有/  ，指代/usr/local/share/pcl-1.12/文件夹下是PCLConfig.cmake文件

sudo apt install systemd-coredump

# make VERBOSE=1

尝试cmake直接编译ros2包，如果有问题？

# 单步debug

lldb 插件vscode-lldb
llvm

```json
vscode ros_debug 设置
//安装sudo apt-get install python3-catkin-tools
//catkin_tools
参考：https://answers.ros.org/question/313371/vscode-debug-cpp-ros-node/
https://haoqchen.site/2019/08/15/debug-ros-with-vscode/
https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md
记得编译是要catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo或者Debug


