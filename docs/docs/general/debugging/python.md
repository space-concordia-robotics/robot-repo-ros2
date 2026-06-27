# Python Debugging

The typical python debugging approach involves using print statements, however it is also recommended to get familiar with a debugger.

## Debugger

You can run ROS python nodes with pdb by adding `--prefix 'python -m pdb'` to your `ros2 run` command.
Use the `run` command to start program execution.

In the code that you want to debug, if you insert a call to the `breakpoint()` method, and `pdb` will pause execution at that location.

Here are some of the commands you can then use to help debug the program:

- `n(ext)`: Continue execution until the next line in the current function is reached or it returns.
- `s(tep)`: Execute the current line, stop at the first possible occasion, possibly stepping into a function.
- `c(ont(inue))`: Continue execution, only stop when a breakpoint is encountered.
- `l(ist) [first[, last]]`: List source code for the current file. Without arguments, list 11 lines around the current line or continue the previous listing.
- `a(rgs)`: Print the arguments of the current function and their current values.
- `q(uit)`: Quit from the debugger.
- `p expression`: Evaluate *expression* in the current context and print its value.

The full list of commands can be found [here](https://docs.python.org/3/library/pdb.html#pdbcommand-help).

## Address Sanitizer, Thread Sanitizer, and Memory Sanitizer

TODO
