# Linux Commands

Almost all Linux commands have a `--help` flag[^help-flag].
If you are not sure how to use a given command, you can always just do `[command] --help` where `[command]` is the command.

[^help-flag]: sometimes instead of `--help`, some programs like to be contrarian and use `-help` or `-h`.
If `--help` does not work, you can always try one of those.
However, it will usually tell you if this is the case.

## Useful Commands

### `man`

!!! info inline end

    The manual page for something is often called a "man page"

The `man` command shows you the manual for the next argument.
For example, to see the manual for the `ls` command, you would do `man ls`.

The Linux kernel uses man pages extensively for all of its API,
so if you are ever developing something against the kernel's api, you will likely want to reference that.
For example, if you are using linux sockets, you may want to reference the man page for `sys/socket.h` which can be found using `man sys_socket.h`.
