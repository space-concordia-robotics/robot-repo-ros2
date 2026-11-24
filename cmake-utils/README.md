# CMake Utilities

This package contains miscellaneous CMake utilities, which are used by other packages.
It does not do anything on its own, and exists purely to help simplify writing the `CMakeLists.txt` files in other packages.

In addition to several functions, this module does the following things:

1. Automatically tries to add a dependency on [Backward ROS](https://github.com/pal-robotics/backward_ros)
2. Adds some global compiler flags when in debug mode (currently `-g3` and `-ggdb3`).

## Provided Functions

The following functions are provided by this module:

- `find_dependencies([<scope>] <dependency>... [<scope> <dependency>...]...)`: Finds the provided dependencies with `find_package()`.\
  Where `scope` is one of: `INTERFACE`, `PUBLIC`, or `PRIVATE`.\
  After this is invoked, the following variables will be set in the caller's scope:
    - `INTERFACE_DEPENDENCIES`: a list of the interface dependencies.
    - `PUBLIC_DEPENDENCIES`: a list of the public dependencies.
    - `PRIVATE_DEPENDENCIES`: a list of the private dependencies.
    - `DEPENDENCIES`: a list of all dependencies.
    - `INTERFACE_DEPENDENCY_TARGETS`: a list of the interface dependency targets.
    - `PUBLIC_DEPENDENCY_TARGETS`: a list of the public dependency targets.
    - `PRIVATE_DEPENDENCY_TARGETS`: a list of the private dependency targets.
    - `DEPENDENCY_TARGETS`: a list of all dependency targets.

### Compiler Defaults

The following functions are provided for configuring compiler defaults:

- `scrb_add_executable(<name> <options>... <sources>...)`: Replacement for CMake's `add_executable()` that configures compiler defaults for the target.\
  Additional options:
    - `NO_SANITIZERS`: Disables the use of sanitizers for this target.
- `scrb_add_library(<name> [<type>] <options>...> <sources>...)`: Replacement for CMake's `add_library()` that configures compiler defaults for the target.\
  Additional options:
    - `NO_SANITIZERS`: Disables the use of sanitizers for this target.
- `apply_target_defaults(<name> [NO_SANITIZERS])`: Applies compiler defaults to a target, useful when using `ament_add_gtest()`.
    - `NO_SANITIZERS`: Disables the use of sanitizers for this target.

The above functions configure the target with the following compiler defaults:

- Enabling a bunch of compiler warnings.
    - Enables: `-Wall`, `-Wextra`, and several additional warnings (see `compiler-defaults.cmake` for the full details)
    - Enables a few additional GCC & Clang specific warnings based on the compiler in use.
- Enables sanitizers.
- Enables LTO when compiling in release mode.
- Targets C++23 & C23

## CPM

[CPM](https://github.com/cpm-cmake/CPM.cmake) is used to download source dependencies. The following CPM-related functions are provided:

- `CPM()`: Includes CPM in the current build.
- `CPMAddFmt()`: Adds a CPM package for [fmt](https://fmt.dev/)
- `CPMAddMagicEnum()`: Adds a CPM package for [Magic Enum](https://github.com/Neargye/magic_enum)

## Options

When including this module, the following CMake options are added:

- `ENABLE_TSAN`: if the [tsan](https://clang.llvm.org/docs/ThreadSanitizer.html) should be enabled, defaults to `OFF`.
  Enabling this will disable asan and lsan.
- `ENABLE_SANITIZERS`: if sanitizers should be enabled, defaults to `ON`.
  The enabled sanitizers always includes [ubsan](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html).
  If `ENABLE_TSAN` is set to `OFF`, then [asan](https://clang.llvm.org/docs/AddressSanitizer.html) and [lsan](https://clang.llvm.org/docs/LeakSanitizer.html)
  will be enabled, otherwise tsan will be enabled.
- `FORCE_SANITIZERS`: if the sanitizers should be enabled even for targets that have opted out.
  Targets that opt out of the sanitizers are generally shared libraries that need to be `dlopen`ed (e.g. anything that
  uses [pluginlib](https://github.com/ros/pluginlib/), such as a ros2_control hardware interface).
  If you want to use a sanitizer for a dynamic shared library, then you must `LD_PRELOAD` asan.
  If you know what any of these words mean, then you know how to do this.
  Otherwise, you probably should not be messing around with this.
- `SCRB_PRECOMPILE_HEADERS`: if precompiled headers should be used. (also set by the `SCRB_PRECOMPILE_HEADERS` environment variable)
