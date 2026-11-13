# SCRB Rover Code

This repository contains the code for the NVIDIA Jetson for the SCRB software team.

## Getting Started

To get started, please read our online documentation found at: https://space-concordia-robotics.github.io/

This documentation is derived from the `docs/` folder, so you can also read it from within your IDE.

## Building

In order to build the project, you need to first clone all the dependencies:

```bash
vcs import external < external.repos
```

This uses `git clone` to clone all the repositories in `external.repos` to `external`

Then, you can build everything using `colcon`

# License

This project lis licensed under the [MIT license](./LICENSE).
