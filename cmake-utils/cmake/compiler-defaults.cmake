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
        -Wc++-compat
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

function(apply_target_defaults TARGET)
    target_compile_features("${TARGET}" PUBLIC cxx_std_23)
    target_compile_features("${TARGET}" PUBLIC c_std_23)

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options("${TARGET}" PRIVATE ${GCC_WARNINGS})
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options("${TARGET}" PRIVATE ${CLANG_WARNINGS})
    endif ()

    target_compile_options("${TARGET}" PRIVATE -fno-omit-frame-pointer)

    # add asan & ubsan for debug mode
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options("${TARGET}" PRIVATE -fsanitize=address,undefined)
        target_link_options("${TARGET}" PRIVATE -fsanitize=address,undefined)
    endif ()
endfunction()

function(scrb_add_executable TARGET)
    add_executable("${TARGET}" ${ARGN})
    apply_target_defaults("${TARGET}")
endfunction()

function(scrb_add_library TARGET)
    add_library("${TARGET}" ${ARGN})
    apply_target_defaults("${TARGET}")
endfunction()
