include(FetchContent)

find_program(MAKE_EXE NAMES meson)

find_package(Python3 REQUIRED)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(MESON_BUILD_TYPE "debug")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Release")
    set(MESON_BUILD_TYPE "release")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "RelWithDebInfo")
    set(MESON_BUILD_TYPE "debugoptimized")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "MinSizeRel")
    set(MESON_BUILD_TYPE "minsize")
else ()
    set(MESON_BUILD_TYPE "debugoptimized")
endif ()

FetchContent_Declare(
        peel_src
        GIT_REPOSITORY https://gitlab.gnome.org/bugaevc/peel.git
        GIT_TAG main
)

FetchContent_MakeAvailable(peel_src)

file(REAL_PATH "../peel-install" peel_INSTALL_DIR BASE_DIRECTORY "${peel_src_SOURCE_DIR}")
file(MAKE_DIRECTORY "${peel_INSTALL_DIR}")

execute_process(COMMAND meson setup --prefix "${peel_INSTALL_DIR}" "${peel_src_BINARY_DIR}" "${peel_src_SOURCE_DIR}")
execute_process(COMMAND meson compile -C "${peel_src_BINARY_DIR}")
execute_process(COMMAND meson install -C "${peel_src_BINARY_DIR}")

# everything below this line is vendored from peel's peel-config.cmake.
# it was copied on 2026-05-21. if the file has changed since then & you're updating to a newer version, please update this config as well.

set(peel_VERSION "0.0.0")
set(peel_LIBRARIES)
set(peel_INCLUDE_DIRS "${peel_INSTALL_DIR}/include")
set(PEEL_GEN_MODULE "${peel_INSTALL_DIR}/bin/peel-gen")
mark_as_advanced(peel_VERSION peel_LIBRARIES peel_INCLUDE_DIRS PEEL_GEN_MODULE)

add_library(peel::peel INTERFACE IMPORTED)

set_property(TARGET peel::peel APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${peel_INCLUDE_DIRS}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(peel
        REQUIRED_VARS PEEL_GEN_MODULE peel_INCLUDE_DIRS
        VERSION_VAR peel_VERSION)

function(peel_generate GIR_NAME GIR_VERSION)
    # Parse our arguments.
    set(options RECURSIVE OPTIONAL GLOBAL)
    set(oneValueArgs GIR_FILE TWEAKS)
    set(multiValueArgs)
    cmake_parse_arguments(peel_generate "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(gi_gir_path "$ENV{GI_GIR_PATH}")
    foreach (prefix_path IN LISTS CMAKE_PREFIX_PATH)
        file(TO_NATIVE_PATH "${prefix_path}" native_prefix_path)
        if ("${gi_gir_path}" STREQUAL "")
            set(sep)
        elseif (UNIX)
            set(sep ":")
        else ()
            set(sep ";")
        endif ()
        if (UNIX)
            set(gi_gir_path "${gi_gir_path}${sep}${native_prefix_path}/share/gir-1.0")
        else ()
            set(gi_gir_path "${gi_gir_path}${sep}${native_prefix_path}\\share\\gir-1.0")
        endif ()
    endforeach ()

    if (${peel_generate_RECURSIVE})
        set(recursive_arg "--recursive")
    else ()
        set(recursive_arg)
    endif ()

    if (DEFINED peel_generate_TWEAKS)
        set(api_tweaks_arg --api-tweaks "${peel_generate_TWEAKS}")
    else ()
        set(api_tweaks_arg)
    endif ()

    message("api tweaks: ${peel_generate_TWEAKS}")
    message("api tweaks arg: ${api_tweaks_arg}")

    set(gen_dir "${CMAKE_CURRENT_BINARY_DIR}/peel-generated")
    file(MAKE_DIRECTORY "${gen_dir}")

    if (${CMAKE_VERSION} VERSION_LESS "3.27")
        set(depends_explicit_only)
    else ()
        set(depends_explicit_only DEPENDS_EXPLICIT_ONLY)
    endif ()
    if (${CMAKE_VERSION} VERSION_LESS "3.31")
        set(codegen_arg)
    else ()
        cmake_policy(SET CMP0171 NEW)
        set(codegen_arg CODEGEN)
    endif ()

    message("missing values: ${peel_generate_KEYWORDS_MISSING_VALUES}")
    message("command: ${CMAKE_COMMAND} -E env GI_GIR_PATH=${gi_gir_path} ${Python3_EXECUTABLE} ${PEEL_GEN_MODULE} ${recursive_arg} ${api_tweaks_arg} ${GIR_NAME} ${GIR_VERSION}")

    add_custom_command(
            OUTPUT "${gen_dir}/peel/${GIR_NAME}"
            COMMAND "${CMAKE_COMMAND}"
            ARGS
            "-E" "env" "GI_GIR_PATH=${gi_gir_path}"
            "${Python3_EXECUTABLE}" "${PEEL_GEN_MODULE}"
            ${recursive_arg} ${api_tweaks_arg} "${GIR_NAME}" "${GIR_VERSION}"
            DEPENDS "${PEEL_GEN_MODULE}" "${peel_generate_TWEAKS}"
            WORKING_DIRECTORY "${gen_dir}"
            VERBATIM ${depends_explicit_only} ${codegen_arg}
    )

    add_custom_target(peel_generate_${GIR_NAME} DEPENDS "${gen_dir}/peel/${GIR_NAME}")

    add_library(peel::${GIR_NAME} INTERFACE IMPORTED)
    target_link_libraries(peel::${GIR_NAME} INTERFACE peel::peel)
    add_dependencies(peel::${GIR_NAME} peel_generate_${GIR_NAME})

    set_property(TARGET peel::${GIR_NAME} APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${gen_dir}")
endfunction(peel_generate)
