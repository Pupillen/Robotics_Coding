### GProf: non intrusive overall timing.

Official doc with information about how to interpret the output: [https://github.com/gperftools/gperftools/blob/master/docs/cpuprofile.html](https://github.com/gperftools/gperftools/blob/master/docs/cpuprofile.html)

#### Short documentation:

Install google performance tools:

**Ubuntu**

```bash
sudo apt-get install google-perftools libgoogle-perftools-dev graphviz
```

**OSX**
Newer (2018) instructions can be found [here](https://stackoverflow.com/questions/22623934/how-to-install-gprof-on-os-x/49662636#49662636).

```bash
brew install google-perftools graphviz
```

### CPU Profiling

Then when you run your code, you need to define CPUPROFILE and LD_PRELOAD

**Ubuntu**

```bash
CPUPROFILE=/tmp/my_executable.prof LD_PRELOAD=/usr/lib/libprofiler.so.0 ./my_executable
```

**OSX**

```bash
CPUPROFILE=/tmp/my_executable.prof DYLD_INSERT_LIBRARIES=/usr/local/Cellar/google-perftools/2.1/lib/libprofiler.dylib ./my_executable
```

**Ubuntu 18.04**

In Ubuntu 18.04 `libprofiler.so` is locate at:

```bash
/usr/lib/x86_64-linux-gnu/libprofiler.so
```

Finally, generate the call-graph pdf:

**Ubuntu**

```bash
google-pprof --pdf my_executable /tmp/my_executable.prof > my_executable_profiling.pdf
```

**OSX**

```
pprof --pdf my_executable /tmp/my_executable.prof > my_executable_profiling.pdf
```

The call graph then looks like:

***

![](http://google-perftools.googlecode.com/svn/trunk/doc/pprof-test.gif)

***

**CPU Profiling with ROS on Ubuntu**

Profiling with as roslaunch is easy, since all subprocesses get captured automatically.

First, run roslaunch with the profiler:

```
env CPUPROFILE=/tmp/my_executable.prof LD_PRELOAD=/usr/lib/libprofiler.so.0 roslaunch dvs_tracking slam_ros.launch
```

Second, find out the PID of the node you want to profile.
It is printed with every _LOG(INFO)_ command.

Finally, generate the PDF call graph (replace PATH_TO_EXECUTABLE and PID):

```
google-pprof --pdf /home/odroid/catkin_ws/devel/lib/PATH_TO_EXECUTABLE  /tmp/my_executable.prof_PID > profile.pdf
```

**Warning:** Your node has to shut down properly in order to write the profile to disk. If it doesn't, the profile could be empty.

***

Other useful variables:

```
CPUPROFILE_FREQUENCY=x
```

where the default x = 100 (value is in Hz).

### sm::timing: detailed but intrusive timing.

```c++
// Pull in the sm_timing definitions
#include <sm_timing/Timer.h>

//Enable timing.
typedef sm::timing::Timer Timer;

//Disable timing by switching the typedef.
typedef sm::timing::DummyTimer Timer;
```

After adding this definition you can now time specific code segments by adding a timer call:

```c++
Timer expensive_operation_timer("ExpensiveOperation");
DoExpensiveOperation();
expensive_operation_timer.Stop();
```

When you want to print out the timing information you call:

```c++
sm::timing::Timing::Print(std::cout);
//or similarly
LOG(INFO) << sm::timing::Timing::Print();
```

You will get a table containing the following information:

```
SM Timing
---------
//a                 b c               d                 e             f           g           
ExpensiveOperation  5 00:00:09:00.12 (00:00:08:56.67 +- 00:00:00.15) [00:00:08.00 00:00:09.000]
```

Where the columns denote:

a) Name of the timer.  
b) Number of calls.  
c) Overall time of all calls.  
d) Mean time of a call.  
e) Std-dev of the times.  
f) Min time of a call.  
g) Max time of a call.

### Memory profiling

Then when you run your code, you need to define CPUPROFILE and LD_PRELOAD

**Ubuntu**

```bash
HEAPPROFILE=/tmp/mybin.hprof LD_PRELOAD=/usr/lib/libtcmalloc.so.4 ./my_executable
```

**OSX**

```bash
HEAPPROFILE=/tmp/mybin.hprof DYLD_INSERT_LIBRARIES=/usr/local/Cellar/google-perftools/2.1/lib/libtcmalloc.dylib ./my_executable
```

Finally, generate the call-graph pdf:

**Ubuntu**

```bash
google-pprof --pdf my_executable /tmp/profile.0001.heap > my_executable_heap_profiling.pdf
```

**OSX**

```
pprof --pdf my_executable /tmp/profile.0001.heap > my_executable_heap_profiling.pdf
```

Depending on your [settings](https://github.com/gperftools/gperftools/blob/master/docs/heapprofile.html) you will get many *.heap files that are created during the lifetime of your application.