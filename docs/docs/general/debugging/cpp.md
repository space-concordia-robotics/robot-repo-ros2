# C++ Debugging

It is recommended that you read [Getting Backtraces in ROS 2](https://docs.ros.org/en/rolling/How-To-Guides/Getting-Backtraces-in-ROS-2.html).

When debugging ROS nodes, it may be helpful to run them with GDB or LLDB.
You can do this by adding `--prefix 'gdb -ex run --args'` or `--prefix 'lldb -o run --'` to your `ros2 run` command.

If you don't know how to use LLDB, then you can check out the [LLDB tutorial](https://lldb.llvm.org/use/tutorial.html).
The [GDB to LLDB command map](https://lldb.llvm.org/use/map.html) page from the LLDB documentation is also helpful.
For GDB, there is no good official tutorial, however there are many good unofficial ones.

In order to start executing a process when using GDB or LLDB, use `run` (or `r`).
When a process has been paused, continue with `continue` (or `c`) or exit with `quit` (or `q`)

In order for GDB or LLDB to provide any useful info, you will want to have debug symbols.
To build all of the packages with debug symbols, you can use one of the following commands:

```bash
colcon build --symlink-install --mixin rel-with-deb-info # build in release mode with debug info
colcon build --symlink-install --mixin debug # build in debug mode
```

The most generally useful commands in GDB and LLDB are:

## Print the stack backtrace

=== "GDB"

    ```prefix { prefix="(gdb)" title="GDB" }
    bt
    ```

=== "LLDB"

    ```prefix { prefix="(lldb)" title="LLDB" }
    thread backtrace
    bt
    ```

## Show the local variables

=== "GDB"

    ```prefix { prefix="(gdb)" title="GDB" }
    info args
    info locals
    ```

=== "LLDB"

    ```prefix { prefix="(lldb)" title="LLDB" }
    frame variable
    ```
