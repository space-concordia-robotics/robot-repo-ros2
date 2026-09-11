# Code Standards

It is necessary that everyone follow the same coding standards in order for everything to not become a mess.
As such, this guideline outlines the set of standards that everyone is expected to follow.

## General

### Naming

- All packages are named using `kebab-case` for the folder name, however the actual package name itself is named using `snake_case`. For example, a package with
  the directory `arm-controller` would have the name `arm_controller`.
  Note that sometimes the folder name does not always reflect the exact name of the package (but it should be similar),
  for example the `scrb_common_util` package has the folder name `common-util`,
  this is because the `scrb` part is used to differentiate it from everything else but that is not necessary in the file tree.
- Scripts should be named in `snake_case`

### Formatting

Four-space indentation should be used for everything (except YAML, which uses two-space indentation), unless the format disallows it.

Before committing your code, you should use a code formatter to ensure everything is properly formatted.

???+ tip "Run your IDE's built-in formatter"

    For vscode, you can run the built-in formatter with ++shift+alt+f++ on Windows, ++shift+option+f++ on macOS, or ++ctrl+shift+i++ on Linux.

    For CLion, you can run the built-in formatter with ++ctrl+alt+l++.

### Code cleanliness

It is generally recommended to avoid extremely long functions. Extremely long functions should generally be broken up into smaller logical units.

There is no good universally accepted standard for what is considered an "extremely long" function, as sometimes functions just need to "do a lot".
This is a skill you will pick up over time, however you will get feedback during code reviews which will help with this.

## C++

### Naming

In order to keep everything consistent, we follow the following naming conventions:

- classes, structs, enums, and other types must be named in `PascalCase`
- fields must be named in `snake_case`
- functions in classes must be named in `camelCase`
- top-level functions must be named in `snake_case`
- constants (`#!cpp static constexpr`) must be named in `UPPER_SNAKE_CASE`
- files & modules use `snake_case`

### Modern C++ Features

When working in C++, you should use modern C++ features, for example:

- [`#!cpp auto`](https://en.cppreference.com/cpp/language/auto)
- [Brace initializers](https://en.cppreference.com/cpp/language/list_initialization) & [Designated initializers](https://en.cppreference.com/cpp/language/aggregate_initialization#Designated_initializers)
- [Lambda expressions](https://cppreference.com/cpp/language/lambda)
- [Smart pointers](https://cppreference.com/cpp/memory)
- [range for loops](https://en.cppreference.com/cpp/language/range-for)
- [Structured bindings](https://en.cppreference.com/cpp/language/structured_binding)
- [Coroutines](https://en.cppreference.com/cpp/language/coroutines)
- [Constraints & Concepts](https://en.cppreference.com/cpp/language/constraints)
- [`std::optional`](https://en.cppreference.com/cpp/utility/optional)
- [`std::variant`](https://en.cppreference.com/cpp/utility/variant)
- [`std::ranges`](https://en.cppreference.com/cpp/header/ranges)
- [`std::chrono`](https://en.cppreference.com/cpp/header/chrono)
- [`std::numbers`](https://en.cppreference.com/cpp/header/numbers)
- & more

### Avoid C-isms

You should try to avoid C-isms, for example:

- `#!cpp #define FOO 1`, prefer `#!cpp static constexpr auto FOO = 1;`
- pointers for arrays, prefer `std::vector`, `std::span`, or `std::array`
- manual memory management (`malloc`, `free`, etc.)
- C-style strings (`#!cpp char*`, etc.), prefer `std::string` or `std::string_view`
- C-style casts, prefer `#!cpp static_cast`, `#!cpp reinterpret_cast`, etc.
- C numerical constants (e.g. `M_PI`, etc.), prefer `std::numbers`
- macros, prefer templates (though sometimes macros are unavoidable)
- `#!cpp typedef`, prefer `#!cpp using`
- C-style enums, prefer `#!cpp enum class`
- sentinel values (e.g. returning `#!cpp -1` or `#!cpp nullptr` for error/unknown), prefer `std::optional` or `std::expected`
- global state (mutable top-level variables), prefer encapsulation in an object that is passed around.
- `memcpy`
- untyped durations (e.g. a parameter in milliseconds which takes an `#!cpp int`), prefer `std::chrono`
- header/include guards, prefer `#!cpp #pragma once`
- the `#!cpp []` operator for arrays, vectors, etc., prefer `#!cpp .at()` (e.g. do `#!cpp vec.at(0)` instead of `#!cpp vec[0]`)
- `#!cpp strerror(errno)`, prefer
  ```cpp title=""
  const auto code = std::make_error_code(static_cast<std::errc>(errno))
  code.message()
  ```
- `#!cpp typedef struct {} Foo`, prefer `#!cpp struct Foo {}`

### Miscellaneous

- Do not add `#!cpp // namespace foo` comments at the end of a namespace.

### Clang-Tidy & Clang-Format

All files should be reformatted using [Clang-Format](https://clang.llvm.org/docs/ClangFormat.html), using the `.clang-format` config in the root of this
repository.

[Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/) should be used to lint all files prior to committing or PRing. Please try to fix all the warnings that it
shows, it shows them for a reason.\
If you really cannot fix the warning, then you can suppress the warning, however you should provide a reason for the suppression, for example:

```cpp
// NOLINTNEXTLINE(*-suspicious-stringview-data-usage): name is null terminated
if (ImGui::Selectable(name.data(), selected)) {
```

### Libraries

In order to keep our code clean, we generally employ certain libraries.

Currently, the list of libraries that we use most often are:

- [{fmt}](https://fmt.dev/): used for string formatting instead of `printf`-style functions
- [ros2_fmt_logger](https://github.com/nobleo/ros2_fmt_logger): used for logging instead of the standard `RCLCPP_` macros
- [magic_enum](https://github.com/Neargye/magic_enum): used for simplifying working with enums (primarily with serializing & deserializing from strings)

You should also try to use the following libraries where possible:

- [Eigen](https://libeigen.gitlab.io/eigen/docs-3.4/)
- [OpenCV](https://docs.opencv.org/4.13.0/index.html)

## Python

### Naming

In order to keep everything consistent, we follow the following naming conventions:

- classes must be named in `PascalCase`
- fields, functions, and methods use `snake_case`
- enum fields use `UPPER_SNAKE_CASE`
- files & packages use `snake_case`

### Type Hints

Type hints should be added to nearly everything.
If it's too complex to add a type hint, then it's probably because you're doing something too complex with it and it should instead be refactored.

### Ruff & ty

[Ruff](https://docs.astral.sh/ruff/) is used for general linting of python code, and should be run on all files prior to a commit or a PR. Please fix any errors
or warnings from ruff.\
If for some reason the warning cannot be fixed or is a false positive, then it can be suppressed like this:

```python
except Exception as e:  # noqa: BLE001
    self._error = str(e)
```

[ty](https://docs.astral.sh/ty/) is then used for type checking python code, this should also be used prior to committing or PRing.\
False positives can also be similarly suppressed:

```python
sum_three_numbers("one", 5)  # ty: ignore[missing-argument, invalid-argument-type]
```

### Launch Files

In order to simplify writing launch files, the `launch_util` package has been created.
Please read [Launch Files](./launch-files.md) for how to use it.
