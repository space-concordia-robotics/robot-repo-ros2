cmake_minimum_required(VERSION 3.25)

option(ENABLE_TSAN "Enable ThreadSanitizer for debug builds" OFF)
option(SCRB_PRECOMPILE_HEADERS "Enable precompiled headers" ON) # due to https://youtrack.jetbrains.com/issue/CPP-49453/, you need to disable these for CLion

if (DEFINED ENV{SCRB_PRECOMPILE_HEADERS})
    if (NOT $ENV{SCRB_PRECOMPILE_HEADERS})
        set(SCRB_PRECOMPILE_HEADERS OFF CACHE BOOL "Enable precompiled headers" FORCE)
    endif ()
endif ()

include(CheckCompilerFlag)
include(CheckLinkerFlag)

set(CLANG_GCC_SHARED_WARNINGS
        -Wall
        -Wextra             # reasonable and standard
        -Wpedantic          # warn if non-standard C++ is used
        -Wnon-virtual-dtor  # warn the user if a class with virtual functions has a non-virtual destructor
        -Wcast-align        # warn for potential performance problem casts
        -Wnull-dereference  # warn if a null dereference is detected
        -Wformat=2          # warn on security issues around functions that format output (ie printf)
        -Wdeprecated        # warn on all deprecated language features
        -Wpragmas           # warn on bad use of pragma
)

set(CLANG_WARNINGS
        ${CLANG_GCC_SHARED_WARNINGS}
        -Wgnu
        -Wno-c2y-extensions # disable c2y warnings because ROS uses __COUNTER__
)

set(GCC_WARNINGS
        ${CLANG_GCC_SHARED_WARNINGS}
        -Wduplicated-cond           # warn if if/else chain has duplicated conditions
        -Wduplicated-branches       # warn if if/else branches have duplicated code
        -Wlogical-op                # warn about logical operations being used where bitwise were probably wanted
        -Wuseless-cast              # warn if you perform a cast to the same type
)

# add to CMAKE_C_FLAGS and CMAKE_CXX_FLAGS for Debug & RelWithDebInfo builds
function(add_global_debug_flag FLAG)
    check_compiler_flag(C "${FLAG}" HAS_C_FLAG)

    if (HAS_C_FLAG)
        string(APPEND CMAKE_C_FLAGS_DEBUG " ${FLAG}")
        string(APPEND CMAKE_C_FLAGS_RELWITHDEBINFO " ${FLAG}")
    endif ()

    if (CMAKE_CXX_COMPILER)
        check_compiler_flag(CXX "${FLAG}" HAS_CXX_FLAG)
        if (HAS_CXX_FLAG)
            string(APPEND CMAKE_CXX_FLAGS_DEBUG " ${FLAG}")
            string(APPEND CMAKE_CXX_FLAGS_RELWITHDEBINFO " ${FLAG}")
        endif ()
    endif ()
endfunction()

# add additional debug info
add_global_debug_flag("-g3")
add_global_debug_flag("-ggdb3")
#add_global_debug_flag("-glldb") # tune debugger information for lldb (clang only)

function(append_compiler_flags COMPILE_LIST_VAR)
    foreach (FLAG ${ARGN})
        string(MAKE_C_IDENTIFIER "HAS_COMPILER_FLAG_${FLAG}" COMPILER_VAR_NAME)

        check_compiler_flag(CXX "${FLAG}" "${COMPILER_VAR_NAME}")

        if (${COMPILER_VAR_NAME})
            list(APPEND ${COMPILE_LIST_VAR} "${FLAG}")
        endif ()
    endforeach ()

    return(PROPAGATE ${COMPILE_LIST_VAR})
endfunction()


function(append_linker_flags LINK_LIST_VAR)
    foreach (FLAG ${ARGN})
        string(MAKE_C_IDENTIFIER "HAS_LINKER_FLAG_${FLAG}" LINKER_VAR_NAME)

        check_linker_flag(CXX "${FLAG}" "${LINKER_VAR_NAME}")

        if (${LINKER_VAR_NAME})
            list(APPEND ${LINK_LIST_VAR} "${FLAG}")
        endif ()
    endforeach ()

    return(PROPAGATE ${LINK_LIST_VAR})
endfunction()

function(append_sanitizer_flags COMPILE_LIST_VAR LINK_LIST_VAR)
    set(SANITIZERS "")

    foreach (SANITIZER ${ARGN})
        check_compiler_flag(CXX "-fsanitize=${SANITIZER}" "HAS_COMPILER_${SANITIZER}_FLAG")
        check_linker_flag(CXX "-fsanitize=${SANITIZER}" "HAS_LINKER_${SANITIZER}_FLAG")

        if (HAS_COMPILER_${SANITIZER}_FLAG OR HAS_LINKER_${SANITIZER}_FLAG)
            list(APPEND SANITIZERS "${SANITIZER}")
        else ()
            message(WARNING "Compiler ${CMAKE_CXX_COMPILER_ID} does not support sanitizer ${SANITIZER}, skipping")
        endif ()
    endforeach ()

    if (SANITIZERS)
        # sadly, it seems these all need to be concatenated, otherwise you get linker errors
        # it seems you can't just do -fsanitize=address -fsanitize=undefined
        string(REPLACE ";" "," SANITIZER_STRING "${SANITIZERS}")
        set(FINAL_FLAG "-fsanitize=${SANITIZER_STRING}")

        list(APPEND ${COMPILE_LIST_VAR} "${FINAL_FLAG}")
        list(APPEND ${LINK_LIST_VAR} "${FINAL_FLAG}")
    endif ()

    return(PROPAGATE ${COMPILE_LIST_VAR} ${LINK_LIST_VAR})
endfunction()

function(apply_target_defaults TARGET)
    # enable C++23
    target_compile_features("${TARGET}" PUBLIC cxx_std_23)
    target_compile_features("${TARGET}" PUBLIC c_std_23)

    # enable more aggressive warnings
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options("${TARGET}" PRIVATE ${GCC_WARNINGS})
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options("${TARGET}" PRIVATE ${CLANG_WARNINGS})
    else ()
        message(WARNING "Unknown compiler ${CMAKE_CXX_COMPILER_ID}, are you using MSVC?")
    endif ()

    # enable LTO in release mode
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT error)
    if (ipo_supported)
        set_target_properties("${TARGET}" PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    else ()
        message(WARNING "LTO is not supported for the chosen compiler/linker")
    endif ()

    set(DEBUG_COMPILE_FLAGS "")
    set(DEBUG_LINK_FLAGS "")

    # make debugging easier
    append_compiler_flags(DEBUG_COMPILE_FLAGS
            -fno-omit-frame-pointer     # keep frame pointers
            -fno-optimize-sibling-calls # don't optimize sibling & tail recursion
            -fno-ipa-icf                # disable identical code folding for functions (GCC only)
    )

    # sanitizers for debug mode
    set(SANITIZERS "undefined")
    if (ENABLE_TSAN)
        list(APPEND SANITIZERS "thread")
    else ()
        # add asan & lsan
        list(APPEND SANITIZERS "address" "leak")
    endif ()

    append_sanitizer_flags(DEBUG_COMPILE_FLAGS DEBUG_LINK_FLAGS ${SANITIZERS})

    target_compile_options("${TARGET}" PRIVATE
            $<$<CONFIG:Debug>:${DEBUG_COMPILE_FLAGS}>
            $<$<CONFIG:RelWithDebInfo>:${DEBUG_COMPILE_FLAGS}>
    )
    target_link_libraries("${TARGET}" PUBLIC
            $<$<CONFIG:Debug>:${DEBUG_LINK_FLAGS}>
            $<$<CONFIG:RelWithDebInfo>:${DEBUG_LINK_FLAGS}>
    )
endfunction()

function(scrb_add_executable TARGET)
    add_executable("${TARGET}" ${ARGN})
    apply_target_defaults("${TARGET}")
endfunction()

function(scrb_add_library TARGET)
    add_library("${TARGET}" ${ARGN})
    apply_target_defaults("${TARGET}")
endfunction()

function(scrb_target_precompile_headers TARGET_NAME)
    if (SCRB_PRECOMPILE_HEADERS)
        target_precompile_headers(${TARGET_NAME} ${ARGN})
    endif ()
endfunction()
