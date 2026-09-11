# colcon

## Mixins

`colcon` mixins are extremely useful, as they make it easier to specify some command line options, which are longer or harder to remember.

Everyone should add the default colcon mixin repository:

```bash
colcon mixin add default "https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml"
colcon mixin update default
```

Then, if you want to use a specific mixin, for example using `clang` instead of `gcc` to compile C++ code and `mold` instead of `ld`, you can do:

```bash
colcon build --mixin clang mold
```

Some useful mixins:

- `compile-commands`: creates `compile_commands.json`, which is useful for IDEs like clion.
- `mold`: use the faster `mold` linker instead of `ld`.
- `clang`: use the faster `clang` compiler instead of `gcc`.
- `ninja`: use the `ninja` build backend for `cmake` instead of `make`
- `rel-with-deb-info`: creates a release build with debug info
- `min-size-rel`: creates a minimum size release build
- `release`: creates a release build
- `debug`: creates a debug build

## Selecting build targets

colcon is able to select a specific package to build, which can be done with:

```bash
colcon build --packages-select build-this-package
```

or if you want to build a package and all of its dependencies:

```bash
colcon build --packages-upto build-this-package
```

colcon is also able to exclude specific packages from being built:

```bash
colcon build --packages-skip skip-this-package
```
