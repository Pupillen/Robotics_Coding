# Setup

    sudo apt-get install gdb gdbserver oprofile valgrind
    sudo apt-get install linux-tools-`uname -r` # perf


## Running ROS node within gdb or valgrind (http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)

***************************************************************************************************
- Attach gdbserver to ROS node
- Allows remote debugging

http://wiki.eclipse.org/CDT/User/FAQ#How_do_I_debug_a_remote_application.3F

http://doc.qt.io/qtcreator/creator-debugger-operating-modes.html

     launch-prefix="gdbserver localhost:1337"


***************************************************************************************************
Attach gdb to ROS node

    launch-prefix="terminator -mx gdb -ex run --args"


***************************************************************************************************
- Attach perf to ROS node
- Output in ~/.ros/
- Compile node with -DCMAKE_BUILD_TYPE=RelWithDebInfo for easier results analysis
- add -fno-omit-frame-pointer to CXXFLAGS if you need to record call graphs


    catkin config --append-args --cmake-args -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer"

if you already have -DCMAKE_CXX_FLAGS defined use the command bellow to remove existing configuration (and add the existing flags to the command above)

    catkin config --remove-args -DCMAKE_CXX_FLAGS="..."

## light profiling

    launch-prefix="perf record --verbose --output=perf.out.node_name.data --"

## profiling with call stack calls

    launch-prefix="perf record -g --output=perf.out.node_name.data --"

## profiling with detailed call stack calls

    launch-prefix="perf record -g --call-graph dwarf --output=perf.out.node_name.data --"

## verbose profiling

    launch-prefix="perf record --verbose -g --call-graph dwarf --output=perf.out.node_name.data --"

## setup required if used with --all-cpus flag (system wide profiling)

    sudo su
    echo 0 > /proc/sys/kernel/kptr_restrict
    echo -1 > /proc/sys/kernel/perf_event_paranoid


--------------------------
# Analyse profile results

## For light profiling

    perf report --show-nr-samples --verbose --input=perf.out.node_name.data

## With call graph
    perf report --show-nr-samples --verbose --show-total-period -g 'graph,0.5,callee' --input=perf.out.node_name.data
    perf report --show-nr-samples --verbose --show-total-period -g 'fractal,0.5,callee' --input=perf.out.node_name.data

## For CSV output
    perf report --show-nr-samples --verbose --field-separator=, --show-total-period --show-info --group --demangle --stdio --input=perf.out.node_name.data > perf.out.node_name.data.csv

## Source code with profilling info
    perf annotate --source --input=perf.out.node_name.data [function/symbol name to annotate]

## TUI Shortcuts

a -> show timers alonside code (annotate)


// Attach operf to ROS node
// Output in ~/.ros/
// Use <opreport> to see profile results
    launch-prefix="operf --callgraph --lazy-conversion"

--------------------------
// Analyse profile results

# Report with call graph
    opreport --debug-info --demangle smart --callgraph --global-percent --output-file oprofile_report.txt

# Source code with profilling info
    opannotate --source --demangle smart --output-dir oprofile_annotate


// Attach callgrind to ROS node for profilling
// Output file in ~/.ros/  Can be opened with http://kcachegrind.sourceforge.net/html/Home.html
    launch-prefix="terminator -mx valgrind --tool=callgrind --callgrind-out-file=callgrind.out.node_name.%p"


// Attach massif to ROS node to monitor memory usage
// Output file in ~/.ros/
    launch-prefix="terminator -mx valgrind --tool=massif --heap=yes --stacks=yes --massif-out-file=massif.out.node_name.%p"


// Attach memcheck to ROS node to check for memory leaks
// Output file in ~/.ros/
    launch-prefix="terminator -mx valgrind --tool=memcheck --leak-check=full --show-reachable=yes -v --track-origins=yes --log-file=memcheck.out.node_name.%p"



-----------------------------------------------------------------------------------------------------------------------------------------------------
// To execute the nodes in a different window, prepend terminator -mx to the launch prefixes commands
// Example:
// Attach gdb to ROS node in a different console window
    launch-prefix="terminator -mx gdb -ex run --args"

// Run ROS node in different window to isolate its output from the other nodes
    launch-prefix="terminator -mx"
