cmake_minimum_required(VERSION 3.25)

# this is a macro so that the caller inherits all the variables
# this is required so that all the variables find_package defines get propagated to the caller
macro(find_dependencies)
    set(PUBLIC_DEPENDENCIES "")
    set(PRIVATE_DEPENDENCIES "")
    set(INTERFACE_DEPENDENCIES "")
    set(DEPENDENCIES "")

    set(PUBLIC_DEPENDENCY_TARGETS "")
    set(PRIVATE_DEPENDENCY_TARGETS "")
    set(INTERFACE_DEPENDENCY_TARGETS "")
    set(DEPENDENCY_TARGETS "")

    set(current_scope "PUBLIC")
    set(dependencies "${ARGN}")
    foreach (dependency IN LISTS dependencies)
        if ("${dependency}" STREQUAL "PUBLIC" OR "${dependency}" STREQUAL "PRIVATE" OR "${dependency}" STREQUAL "INTERFACE")
            set(current_scope "${dependency}")
            continue()
        endif ()

        separate_arguments(dependency)

        list(POP_FRONT dependency name)
        find_package(${name} REQUIRED ${dependency})

        set(targets "")

        # I think this checks out? I yoinked it from ament_target_dependencies()
        set(use_modern_cmake FALSE)
        if (NOT "${${name}_TARGETS}" STREQUAL "")
            foreach (_target ${${name}_TARGETS})
                # only use actual targets
                # in case a package uses this variable for other content
                if (TARGET "${_target}")
                    list_append_unique(targets ${_target})
                    set(use_modern_cmake TRUE)
                endif ()
            endforeach ()
        endif ()
        if (NOT use_modern_cmake)
            # otherwise use the classic CMake variables
            foreach (library ${${name}_LIBRARIES})
                if (NOT "${${name}_LIBRARY_DIRS}" STREQUAL "")
                    if (NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
                        unset(lib CACHE)
                        find_library(lib NAMES ${library} PATHS ${${name}_LIBRARY_DIRS} NO_DEFAULT_PATH)
                        if (lib)
                            set(library ${lib})
                        endif ()
                    endif ()
                endif ()
                list(APPEND targets ${library})
            endforeach ()
        endif ()

        list(APPEND targets ${${name}_TARGETS})

        if ("${current_scope}" STREQUAL "PUBLIC")
            list(APPEND PUBLIC_DEPENDENCIES "${name}")
            list(APPEND PUBLIC_DEPENDENCY_TARGETS ${targets})
        elseif ("${current_scope}" STREQUAL "PRIVATE")
            list(APPEND PRIVATE_DEPENDENCIES "${name}")
            list(APPEND PRIVATE_DEPENDENCY_TARGETS ${targets})
        elseif ("${current_scope}" STREQUAL "INTERFACE")
            list(APPEND INTERFACE_DEPENDENCIES "${name}")
            list(APPEND INTERFACE_DEPENDENCY_TARGETS ${targets})
        endif ()

        list(APPEND DEPENDENCIES "${name}")
        list(APPEND DEPENDENCY_TARGETS ${targets})
    endforeach ()

    # clean up variables
    unset(dependencies)
    unset(dependency)
    unset(name)
    unset(targets)
    unset(use_modern_cmake)
    unset(library)
endmacro()
