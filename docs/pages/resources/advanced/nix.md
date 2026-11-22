# Nix/NixOS

!!! warning inline end

    I have just copied my flake onto this page, and have not gone through and cleaned it up.
    There may be things in there that do not work for you.
    It may be outdated and some of the things no longer necessary.
    You are on your own here.

If you are using another Linux distro and do not wish to use Ubuntu,
another alternative is to use [nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay) as a flake.

This **IS NOT RECOMMENDED** for anyone who does not have a significant amount of experience using Linux, as you will likely run into issues that are much harder
to debug than if you were just using Ubuntu.

There is not a lot of documentation on doing this, so you will largely be on your own. You will break things, and will need to figure it out yourself.

However here is something that looks roughly like the flake that I (Will Free) use:

??? node "`flake.nix`"

    ```nix
    {
        inputs = {
            nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
            nixpkgs.follows = "nix-ros-overlay/nixpkgs";    # IMPORTANT!!!
            nixgl.url = "github:tom-ainc/nixgl"; # temporary until upstream fixes issue with nixgl on latest nvidia driver (then revert to nix-community)
        };
        outputs = { self, nix-ros-overlay, nixpkgs, nixgl }:
            nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
                let
                    pkgs = import nixpkgs {
                        inherit system;
                        overlays = [ nix-ros-overlay.overlays.default nixgl.overlay ];
                        config.permittedInsecurePackages = [
                            "freeimage-3.18.0-unstable-2024-04-18"
                        ];
                        config.allowBroken = true;
                        #config.allowInsecure = true;
                        #config.replaceStdenv = ({ pkgs }: pkgs.clangStdenv);
                    };
                    llvm = pkgs.llvmPackages_latest;
                    jazzy = pkgs.rosPackages.jazzy.overrideScope (rosSelf: rosSuper:
                    let
                        clangMoldStdenv = pkgs.stdenvAdapters.useMoldLinker pkgs.clangStdenv;
                        buildRosPackageClang = rosSelf.buildRosPackage.override {
                            stdenv = clangMoldStdenv;
                        };
                        overrideStdenv = (package: package.override {
                            buildRosPackage = buildRosPackageClang;
                        }).overrideAttrs ({
                            propagatedBuildInputs ? [],
                            ...
                        } : {
                            propagatedBuildInputs = propagatedBuildInputs ++ [ llvm.openmp ];
                        });
                    in {
                        rtabmap = with pkgs; with rosSelf; (rosSuper.rtabmap.override {
                            stdenv = clangMoldStdenv;
                        }).overrideAttrs ({
                            cmakeFlags ? [],
                            propagatedBuildInputs ? [],
                            nativeBuildInputs ? [],
                            ...
                        } : {
                            cmakeFlags = cmakeFlags ++ [
                                (lib.cmakeBool "WITH_CERES" true)
                                (lib.cmakeBool "WITH_DEPTHAI" true)
                                (lib.cmakeBool "WITH_PYTHON" true)
                                # todo: odometry approaches, camera drivers, reconstruction approaches, solvers, optional dependencies
                            ];

                            # a whole bunch of optional deps
                            propagatedBuildInputs = propagatedBuildInputs ++ [
                                llvm.openmp

                                # optional dependencies
                                pdal

                                # solvers
                                gtsam
                                ceres-solver
                                libpointmatcher
                                opengv

                                # reconstruction approaches
                                #grid-map
                                #grid-map-core # why can it not find grid map core, and then it also proceeds to fail the compilation in a completely different way???
                                depthai

                                # python
                                python3
                                python3Packages.pybind11
                            ];

                            nativeBuildInputs = nativeBuildInputs ++ [ (lib.lowPrio pkgs.gcc) ];
                        });

                        fp16-luxonis = with pkgs; stdenv.mkDerivation (finalAttrs: {
                            pname = "fp16-luxonis";
                            version = "0.0.0";

                            src = fetchFromGitHub {
                                owner = "luxonis";
                                repo = "FP16";
                                rev = "c911175d2717e562976e606c6e5f799bf40cf94e";
                                hash = "sha256-4U5WmqqljHYoKdKqtFRBX++vGCv/3weuqPFr4WG7GNM=";
                            };

                            nativeBuildInputs = [ cmake ];
                            propagatedBuildInputs = [ /* psimd */ ];

                            cmakeFlags =
                            let
                                psimd = fetchFromGitHub {
                                    owner = "Maratyszcza";
                                    repo = "psimd";
                                    rev = "072586a71b55b7f8c584153d223e95687148a900";
                                    hash = "sha256-lV+VZi2b4SQlRYrhKx9Dxc6HlDEFz3newvcBjTekupo=";
                                };
                            in [
                                (lib.cmakeBool "FP16_BUILD_TESTS" false)
                                (lib.cmakeBool "FP16_BUILD_BENCHMARKS" false)
                                (lib.cmakeBool "FP16_USE_SYSTEM_LIBS" true)
                                (lib.cmakeFeature "PSIMD_SOURCE_DIR" ''${psimd}'')
                            ];

                            doCheck = true;
                        });

                        libarchive-luxonis = with pkgs; pkgs.libarchive.overrideAttrs ({
                            src,
                            postPatch ? "",
                            patches ? [],
                            ...
                        }: {
                            version = "3.5.2";

                            src = fetchFromGitHub {
                                owner = "luxonis";
                                repo = "libarchive";
                                rev = "45baa3a3e57104519e1165bcd5ac29c3bd8c9f3a";
                                hash = "sha256-6KTBpL1ibQAwRdzcan+qPhV5cNPHlxwhPJ+swOwJ92g=";
                            };

                            patches = []; # remove all patches

                            # get rid of hunter
                            postPatch = postPatch + ''
                                sed -i -e '5,9d' CMakeLists.txt
                                sed -i -e '/hunter_add_package/d' CMakeLists.txt
                                sed -i -e 's/\(find_package(.*\) CONFIG/\1/g' CMakeLists.txt
                                sed -i -e '/am_save_LIBS/d' configure.ac # from the patch
                            '';
                        });

                        cpr = with pkgs; clangMoldStdenv.mkDerivation (finalAttrs: {
                            pname = "cpr";
                            version = "1.4.0";

                            src = fetchFromGitHub {
                                owner = "libcpr";
                                repo = "cpr";
                                tag = finalAttrs.version;
                                hash = "sha256-j4AAno3fSBvT5xOx5IsOI9cln2ihp5vpaQvf5uVc6pw=";
                            };

                            patches = [
                                (fetchpatch {
                                    url = "https://github.com/moratom/cpr/commit/50a1321738554e0152b0a6f1b0ca24e4fdecff5c.patch";
                                    hash = "sha256-JinlZ/g2B48E1MujK/22cQNADPX4bFMBnR03+3KPslo=";
                                })
                            ];

                            nativeBuildInputs = [
                                cmake
                                pkg-config
                                gtest
                                cppcheck
                            ];

                            buildInputs = [
                                openssl
                                zlib
                                curl
                            ];

                            cmakeFlags = [
                                # NOTE: Does not build with CPPCHECK or BUILD_CPR_TESTS
                                # (lib.cmakeBool "CPR_ENABLE_CPPCHECK" true)
                                (lib.cmakeBool "BUILD_CPR_TESTS" false)
                                #(lib.cmakeBool "CURL_ZLIB" false)
                                (lib.cmakeBool "BUILD_SHARED_LIBS" true)
                                (lib.cmakeBool "USE_SYSTEM_CURL" true)
                                (lib.cmakeBool "USE_SYSTEM_GTEST" true)
                                (lib.cmakeFeature "CMAKE_BUILD_TYPE" "Release")
                            ];

                            # Install headers
                            postInstall = ''
                                mkdir -p $out/include
                                cp -r $src/include/* $out/include/
                            '';
                        });

                        libnop = with pkgs; stdenv.mkDerivation (finalAttrs: {
                            pname = "libnop";
                            version = "0-unstable-2022-09-04";

                            src = fetchFromGitHub {
                                owner = "luxonis";
                                repo = "libnop";
                                rev = "ab842f51dc2eb13916dc98417c2186b78320ed10";
                                hash = "sha256-d2z/lDI9pe5TR82MxGkR9bBMNXPvzqb9Gsd5jOv6x1A=";
                            };

                            patches = [
                                # System install
                                # https://github.com/luxonis/libnop/pull/6/commits/ae29a8772f38fdb1efc24af9ec2e3f6814eb2158.patch
                                #./001-system-install.patch
                                (fetchpatch {
                                    name = "001-system-install.patch";
                                    url = "https://github.com/luxonis/libnop/pull/6/commits/ae29a8772f38fdb1efc24af9ec2e3f6814eb2158.patch";
                                    hash = "sha256-xhWNno5M7euGdYZuHtESkCs3Zc4NvgLGD0OtB/FbMZM=";
                                })
                                # Fix template warning
                                # https://github.com/luxonis/libnop/pull/6/commits/199978a0fb0dc31de43b80f7504b53958fd202ee.patch
                                #./002-fix-template-warning.patch
                                (fetchpatch {
                                    name = "002-fix-template-warning.patch";
                                    url = "https://github.com/luxonis/libnop/pull/6/commits/199978a0fb0dc31de43b80f7504b53958fd202ee.patch";
                                    hash = "sha256-5FXhO7oRYA+kRRvK9s+90CBX+JEgL7q18d4PFbsXLgQ=";
                                })
                            ];

                            nativeBuildInputs = [ gtest ];

                            # Add optimization flags to address _FORTIFY_SOURCE warning
                            NIX_CFLAGS_COMPILE = [
                                "-O1"
                                "-std=c++17"
                            ];

                            installPhase = ''
                                runHook preInstall
                                make INSTALL_PREFIX=$out install
                                runHook postInstall
                            '';
                        });

                        xlink = with pkgs; clangMoldStdenv.mkDerivation (finalAttrs: {
                            pname = "xlink";
                            version = "0-unstable-2025-14-03";

                            src = fetchFromGitHub {
                                owner = "luxonis";
                                repo = "XLink";
                                rev = "fe8b5450f545a2ebf26dbc093e98c0265d7f4029";
                                hash = "sha256-OTqJfTDudiNrdsDBe1Pg0T1dJcfneGXO/+AIbXpVfxk=";
                            };

                            outputs = [
                                "out"
                                "share"
                            ];

                            # Remove CMake Hunter package manager - needs network connection
                            patches = [
                                #./001-remove-hunter.patch
                                # Bump CMakeLists.txt to 3.10
                                (fetchpatch {
                                    url = "https://github.com/luxonis/XLink/commit/160c6c918c07e28a6a8c5c080a257f7619223304.patch?full_index=1";
                                    hash = "sha256-1VMCteJf/an20fI3UTT/X9cH96dCxPRQolfN+e+6jnU=";
                                })
                            ];

                            postPatch = ''
                                sed -i -e '7,12d' CMakeLists.txt # replacement for 001-remove-hunter.patch
                            '';

                            nativeBuildInputs = [
                                cmake
                                pkg-config
                            ];

                            buildInputs = [ libusb1 ];

                            cmakeFlags = [
                                (lib.cmakeBool "XLINK_ENABLE_LIBUSB" true)
                                (lib.cmakeBool "XLINK_BUILD_EXAMPLES" true)
                                (lib.cmakeBool "XLINK_BUILD_TESTS" true)
                                (lib.cmakeBool "XLINK_LIBUSB_SYSTEM" true)
                                (lib.cmakeFeature "CMAKE_BUILD_TYPE" "Release")
                            ];

                            postInstall = ''
                                mkdir -p $out/include
                                mkdir -p $share/examples
                                mkdir -p $share/tests

                                cp -r $src/include/* $out/include/

                                examples=(
                                    "boot_firmware"
                                    "list_devices"
                                    "boot_bootloader"
                                    "search_devices"
                                    "Makefile"
                                    "device_connect_reset"
                                )

                                tests=(
                                    "multiple_open_stream"
                                    "multithreading_search_test"
                                )

                                find $buildDir
                                for file in "''${examples[@]}"; do
                                    cp examples/$file $share/examples/$file
                                done
                                for file in "''${tests[@]}"; do
                                    cp tests/$file $share/tests/$file
                                done
                            '';
                        });

                        depthai = with pkgs; with rosSelf; let
                            depthai-device-fwp = fetchurl {
                                url = "https://artifacts.luxonis.com/artifactory/luxonis-myriad-snapshot-local/depthai-device-side/a62b2ccb0bc493c2fb41694cb81c08887be24c52/depthai-device-fwp-a62b2ccb0bc493c2fb41694cb81c08887be24c52.tar.xz";
                                hash = "sha256-njwp9WIq0OyEIsHx4hoHhbRAr+W3tl/PnmQ77OBgbZc=";
                            };
                            depthai-bootloader-fwp = fetchurl {
                                url = "https://artifacts.luxonis.com/artifactory/luxonis-myriad-release-local/depthai-bootloader/0.0.28/depthai-bootloader-fwp-0.0.28.tar.xz";
                                hash = "sha256-5xQ7vj0BLt2dgZQKC+Ug6BVkcIt7TsebBx1wF1i7gow=";
                            };
                        in
                        (rosSuper.depthai.override {
                            buildRosPackage = buildRosPackageClang;
                        }).overrideAttrs ({
                            buildInputs ? [],
                            nativeBuildInputs ? [],
                            propagatedBuildInputs ? [],
                            cmakeFlags ? [],
                            ...
                        }: {
                            # evil sed commands
                            postPatch = ''
                                sed -i -e '2a find_package(PkgConfig REQUIRED)' CMakeLists.txt

                                # remove CONFIG from find_package (from hunter)
                                sed -i -e 's/\(find_package(.*\) CONFIG/\1/g' cmake/depthaiDependencies.cmake
                                sed -i -e 's/find_package(archive_static/find_package(LibArchive/g' cmake/depthaiDependencies.cmake
                                sed -i -e 's/find_package(lzma/find_package(LibLZMA/g' cmake/depthaiDependencies.cmake
                                sed -i -e '213,226d' cmake/DepthaiDownloader.cmake
                                sed -i -e '211a file(MAKE_DIRECTORY "''${folder}")' cmake/DepthaiDownloader.cmake
                                sed -i -e '212a file(COPY_FILE "''${DEPTHAI_DEVICE_FWP}" "''${folder}/depthai-device-fwp-''${_version_commit_identifier}.tar.xz")' cmake/DepthaiDownloader.cmake
                                sed -i -e '213a list(APPEND "''${output_list_var}" "''${folder}/depthai-device-fwp-''${_version_commit_identifier}.tar.xz")' cmake/DepthaiDownloader.cmake

                                sed -i -e '189,202d' cmake/DepthaiBootloaderDownloader.cmake
                                sed -i -e '189a file(MAKE_DIRECTORY "''${folder}")' cmake/DepthaiBootloaderDownloader.cmake
                                sed -i -e '190a file(COPY_FILE "''${DEPTHAI_BOOTLOADER_FWP}" "''${folder}/depthai-bootloader-fwp-''${_version_commit_identifier}.tar.xz")' cmake/DepthaiBootloaderDownloader.cmake
                                sed -i -e '191a list(APPEND "''${output_list_var}" "''${folder}/depthai-bootloader-fwp-''${_version_commit_identifier}.tar.xz")' cmake/DepthaiBootloaderDownloader.cmake

                                sed -i -e '752d' CMakeLists.txt # remove file(RELATIVE_PATH command
                                sed -i -e '/HUNTER_INSTALL_PREFIX/d' CMakeLists.txt

                                sed -i -e 's/CURL::libcurl/curl/g' CMakeLists.txt
                                sed -i -e 's/cpr::cpr/cpr/g' CMakeLists.txt
                                sed -i -e 's/lz4::lz4/lz4/g' CMakeLists.txt
                                sed -i -e 's/liblzma::liblzma/liblzma/g' CMakeLists.txt
                                sed -i -e 's/foxglove-websocket::foxglove-websocket/foxglove_websocket/g' CMakeLists.txt
                                sed -i -e '508a find_package(PkgConfig REQUIRED)\npkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)\ntarget_link_libraries(depthai-core PRIVATE PkgConfig::LIBUSB)' CMakeLists.txt
                                sed -i -e 's/usb-1.0/PkgConfig::LIBUSB/g' CMakeLists.txt
                                sed -i -e 's/BZip2::bz2/BZip2::BZip2/g' CMakeLists.txt
                                sed -i -e 's/archive_static/LibArchive::LibArchive/g' CMakeLists.txt
                                sed -i -e 's/ZLIB::zlib/ZLIB::ZLIB/g' CMakeLists.txt
                                sed -i -e '/libnop/d' CMakeLists.txt
                            '';

                            cmakeFlags = cmakeFlags ++ [
                                (lib.cmakeBool "HUNTER_ENABLED" false)
                                (lib.cmakeBool "DEPTHAI_ENABLE_BACKWARD" false) # temporary
                                (lib.cmakeBool "DEPTHAI_ENABLE_CURL" false) # todo: fix cpr
                                (lib.cmakeBool "DEPTHAI_ENABLE_LIBUSB" false) # todo: fix libusb
                                (lib.cmakeFeature "DEPTHAI_DEVICE_FWP" "${depthai-device-fwp}")
                                (lib.cmakeFeature "DEPTHAI_BOOTLOADER_FWP" "${depthai-bootloader-fwp}")
                                (lib.cmakeBool "BUILD_SHARED_LIBS" (!(stdenv.hostPlatform.isStatic)))
                                #(lib.cmakeBool "DEPTHAI_PCL_SUPPORT" true) # todo: figure out pcl support
                            ];

                            buildInputs = buildInputs ++ [ libnop ];
                            nativeBuildInputs = nativeBuildInputs ++ [ pkg-config ];
                            propagatedBuildInputs = propagatedBuildInputs ++ [ bzip2 fp16-luxonis libarchive-luxonis xz zlib spdlog curl cpr ghc_filesystem backward-cpp nlohmann_json libnop xlink opencv opencv.cxxdev libusb1.dev llvm.openmp ];
                        });

                        ros2-fmt-logger = with pkgs; with rosSelf; buildRosPackage {
                            pname = "ros-jazzy-ros2-fmt-logger";
                            version = "1.0.2-beta";

                            src = fetchFromGitHub {
                                owner = "nobleo";
                                repo = "ros2_fmt_logger";
                                rev = "751d0bc32ccd358a8886c7c1e8dc8ebf78fccaf9";
                                hash = "sha256-HQRK/djqdbZuymocIkB1b2ih18HfjRAf4Zj26ACD/0k=";
                            };

                            buildType = "ament_cmake";
                            buildInputs = [ ament-cmake-auto ament-cmake-ros ];
                            checkInputs = [ ament-cmake-gtest ];
                            propagatedBuildInputs = [ ament-index-cpp backward-ros fmt rclcpp rcutils ];
                            nativeBuildInputs = [ ament-cmake-auto ament-cmake-ros ];

                            meta = {
                                description = "A modern, ROS 2 logging library that provides fmt-style formatting as a replacement for RCLCPP logging macros";
                                license = with lib.licenses; [ asl20 ];
                            };
                        };

                        depthai-ros = with rosSelf; rosSuper.depthai-ros.overrideAttrs ({
                            propagatedBuildInputs ? [],
                            ...
                        } : {
                            # remove depthai-examples (why does this not work)
                            propagatedBuildInputs = pkgs.lib.remove depthai-examples propagatedBuildInputs;
                        });

                        # For some reason if I don't vendor this, then it thinks that rviz-marker-tools doesn't exist, even though it definitely does.
                        # not sure why, tbh...
                        rviz-marker-tools = with pkgs; with rosSelf; buildRosPackage {
                            pname = "ros-jazzy-rviz-marker-tools";
                            version = "0.1.4-r3";

                            src = fetchurl {
                                url = "https://github.com/ros2-gbp/moveit_task_constructor-release/archive/release/jazzy/rviz_marker_tools/0.1.4-3.tar.gz";
                                name = "0.1.4-3.tar.gz";
                                sha256 = "f6f6a470feb75d18d4b951a1d4ac2ff64fe36543e850e30b5b287854c2d61b99";
                            };

                            buildType = "ament_cmake";
                            buildInputs = [ ament-cmake urdfdom-headers ];
                            propagatedBuildInputs = [ eigen eigen3-cmake-module geometry-msgs moveit-common rclcpp std-msgs tf2-eigen visualization-msgs ];
                            nativeBuildInputs = [ ament-cmake eigen3-cmake-module ];

                            meta = {
                                description = "Tools for marker creation / handling";
                                license = with lib.licenses; [ bsdOriginal ];
                            };
                        };
                    } // pkgs.lib.mapAttrs (_: package: (package.override {
                            buildRosPackage = buildRosPackageClang;
                        }).overrideAttrs ({
                            propagatedBuildInputs ? [],
                            ...
                        } : {
                            propagatedBuildInputs = propagatedBuildInputs ++ [ llvm.openmp ];
                        })) {
                            # build all of these larger packages with clang because it's faster than gcc

                            rtabmap-sync = rosSuper.rtabmap-sync.overrideAttrs ({
                                hardeningDisable ? [],
                                ...
                            } : {
                                hardeningDisable = hardeningDisable ++ [ "format" ];
                            });

                            rtabmap-viz = with pkgs; rosSuper.rtabmap-viz.overrideAttrs ({
                                nativeBuildInputs ? [],
                                postFixup ? "",
                                ...
                            } : {
                                nativeBuildInputs = nativeBuildInputs ++ [ qt6.wrapQtAppsHook ];
                                dontWrapQtApps = false;

                                postFixup = postFixup + ''
                                    wrapQtApp $out/lib/rtabmap_viz/rtabmap_viz
                                '';
                            });

                            grid-map-core = rosSuper.grid-map-core.overrideAttrs ({
                                NIX_CFLAGS_COMPILE ? "", ...
                            } : {
                                NIX_CFLAGS_COMPILE = NIX_CFLAGS_COMPILE + " " + "-Wno-deprecated-copy-with-dtor";
                            });

                            nav2-costmap-2d = rosSuper.nav2-costmap-2d.overrideAttrs ({
                                NIX_CFLAGS_COMPILE ? "", ...
                            }: {
                                NIX_CFLAGS_COMPILE = NIX_CFLAGS_COMPILE + " " + "-Wno-inconsistent-missing-override";
                            });

                            nav2-collision-monitor = rosSuper.nav2-collision-monitor.overrideAttrs ({
                                NIX_CFLAGS_COMPILE ? "", ...
                            }: {
                                NIX_CFLAGS_COMPILE = NIX_CFLAGS_COMPILE + " " + "-Wno-overloaded-virtual";
                            });

                            inherit (rosSuper)

                            # rtab map
                            rtabmap-conversions
                            rtabmap-demos
                            rtabmap-examples
                            rtabmap-launch
                            rtabmap-msgs
                            rtabmap-odom
                            rtabmap-python
                            rtabmap-ros
                            rtabmap-rviz-plugins
                            rtabmap-slam
                            rtabmap-util

                            gtsam

                            # octomap
                            octomap
                            octomap-mapping
                            octomap-msgs
                            octomap-ros
                            octomap-rviz-plugins
                            octomap-server

                            # grid map
                            grid-map
                            grid-map-cmake-helpers
                            #grid-map-core
                            grid-map-costmap-2d
                            grid-map-cv
                            grid-map-demos
                            grid-map-filters
                            grid-map-loader
                            grid-map-msgs
                            grid-map-octomap
                            grid-map-pcl
                            grid-map-ros
                            grid-map-rviz-plugin
                            grid-map-sdf
                            grid-map-visualization

                            # nav2
                            nav2-amcl
                            nav2-behavior-tree
                            nav2-behaviors
                            nav2-bringup
                            nav2-bt-navigator
                            #nav2-collision-monitor
                            nav2-common
                            nav2-constrained-smoother
                            nav2-controller
                            nav2-core
                            nav2-dwb-controller
                            nav2-graceful-controller
                            nav2-lifecycle-manager
                            nav2-loopback-sim
                            nav2-map-server
                            nav2-msgs
                            nav2-navfn-planner
                            nav2-planner
                            nav2-rviz-plugins
                            nav2-simple-commander
                            nav2-smac-planner
                            nav2-theta-star-planner
                            nav2-util
                            nav2-velocity-smoother
                            nav2-voxel-grid
                            nav2-waypoint-follower
                            nav2-2d-msgs
                            nav2-2d-utils
                            navigation2

                            depthai-bridge
                            depthai-descriptions
                            depthai-filters
                            depthai-ros-driver
                            depthai-ros-msgs
                            ;
                    });
                in {
                    devShells.default = pkgs.mkShell.override {
                        stdenv = pkgs.stdenvAdapters.useMoldLinker pkgs.clangStdenv;
                    } {
                        name = "robot-repo-ros2";
                        packages =
                        let
                            dev-tools = with pkgs; [
                                colcon
                                # (colcon.withExtensions [ python3Packages.colcon-alias python3Packages.colcon-rerun ])

                                gnumake
                                cmake

                                cmake-lint

                                # debugger
                                llvm.lldb
                                gdb

                                clang-tools

                                llvm.libstdcxxClang

                                cppcheck
                                llvm.libllvm
                                valgrind

                                llvm.libcxx
                            ];

                            gstreamer-packages = with pkgs; [
                                gst_all_1.gstreamer
                                gst_all_1.gst-plugins-base
                                gst_all_1.gst-plugins-good
                                gst_all_1.gst-plugins-bad
                                gst_all_1.gst-plugins-ugly
                                gst_all_1.gst-plugins-rs
                                gst_all_1.gst-vaapi
                                gst_all_1.gst-rtsp-server
                                gst_all_1.gst-devtools

                                # why does the rtsp server package need these?
                                libselinux
                                libsepol
                                libsysprof-capture
                                libunwind
                                orc
                            ];

                            simulation-packages = [
                                pkgs.gazebo

                                (with jazzy; buildEnv {
                                    paths = [
                                        ros-gz
                                        ros-gz-image
                                        gz-launch-vendor
                                        gz-ros2-control
                                    ];
                                })
                            ];

                            autonomy-packages = [
                                (with jazzy; buildEnv {
                                    paths = [
                                        perception
                                        nav2-core
                                        nav2-route
                                        nav2-bringup
                                        nav2-rviz-plugins
                                        navigation2
                                        slam-toolbox
                                        robot-localization
                                        #rtabmap
                                        rtabmap-ros

                                        imu-pipeline
                                    ];
                                })
                            ];

                            visualisation-packages = [
                                (with jazzy; buildEnv {
                                    paths = [
                                        rqt
                                        rqt-gui
                                        rqt-common-plugins
                                        rqt-tf-tree
                                        rqt-controller-manager
                                        rqt-image-overlay
                                        rqt-image-overlay-layer
                                        # rqt-moveit
                                        rqt-joint-trajectory-controller
                                        rqt-robot-dashboard
                                        rqt-robot-monitor
                                        rqt-robot-steering

                                        rviz2
                                        rviz-common
                                        rviz-imu-plugin
                                        rviz-marker-tools
                                        rviz-satellite
                                        rviz-visual-tools
                                        rviz-2d-overlay-plugins
                                        battery-state-rviz-overlay
                                        polygon-rviz-plugins
                                        vision-msgs-rviz-plugins

                                        mapviz
                                        mapviz-plugins
                                        tile-map
                                    ];
                                })
                            ];

                            mkdocs-packages = with pkgs; [
                                python3Packages.mkdocs-material
                                python3Packages.mkdocs-material-extensions
                                python3Packages.mkdocs-minify-plugin
                                python3Packages.pygments
                            ];
                        in with pkgs; [
                            (with jazzy; buildEnv {
                                paths = [
                                    ros-core
                                    ament-cmake
                                    ament-cmake-core
                                    #ros-workspace # <--- broken (needs some kind of funny fix with ament?)
                                    #desktop-full # <--- broken
                                    xacro
                                    #ffmpeg-image-transport # <--- broken
                                    urdf-launch

                                    control-msgs
                                    controller-manager
                                    ros2-control
                                    ros2-controllers

                                    camera-info-manager
                                    camera-info-manager-py

                                    ros2-fmt-logger

                                    twist-mux
                                    twist-stamper
                                    teleop-twist-joy
                                    teleop-twist-keyboard

                                    depthai
                                    # for now this is broken because it keeps including depthai-examples
                                    #depthai-ros
                                ];
                            })
                        ] ++ dev-tools ++ gstreamer-packages ++ simulation-packages ++ autonomy-packages ++ visualisation-packages ++ mkdocs-packages;
                    };
                });

        nixConfig = {
            extra-substituters = [ "https://ros.cachix.org" ];
            extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
        };
    }
    ```

I also didn't want to commit this file to the repository, as it's not relevant to others, so I had to do an incredibly nasty workaround because the nix flake
hates not being added to git:\
I have 2 folders, `~/Programming/C++/robot-repo-ros2/` and `~/Programming/C++/robot-repo-ros2.flake/`. the `robot-repo-ros2.flake` folder is a git repo with the
`flake.nix` file in it. And then, the `robot-repo-ros2` folder has a symlink to `flake.nix` inside of it. This stops nix from complaining.
I also have the following `.envrc` file:

```bash
use flake ../robot-repo-ros2.flake --impure

source bin/setup-env.sh
source install/local_setup.bash
```

Some specific things to note:

- the `depthai-ros` package cannot be used because it depends on `depthai-examples`, which tries to fetch something from online and for some reason I can't
  remove it as a dependency, but I cba to figure out why.
- the `depthai` package requires a WHOLE bunch of absolutely disgusting and nasty hacks to get it to work properly without hunter. Do not come complain to me if
  it breaks.
- I will not be explaining this, and if you choose to use this, you will largely be on your own.
  If you can make sense of this, then it may be viable for you to use it.
  Otherwise, I do not recommend using this.
