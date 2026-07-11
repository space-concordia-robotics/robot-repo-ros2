cmake_minimum_required(VERSION 3.25)

function(find_dependencies)
    set(PUBLIC_DEPENDENCIES "")
    set(PRIVATE_DEPENDENCIES "")
    set(INTERFACE_DEPENDENCIES "")
    set(DEPENDENCIES "")

    set(PUBLIC_DEPENDENCY_TARGETS "")
    set(PRIVATE_DEPENDENCY_TARGETS "")
    set(INTERFACE_DEPENDENCY_TARGETS "")
    set(DEPENDENCY_TARGETS "")

    set(current_scope "PUBLIC")
    foreach (dependency IN LISTS ARGN)
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
        if (NOT "${${dependency}_TARGETS}" STREQUAL "")
            foreach (_target ${${dependency}_TARGETS})
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
            foreach (library ${${dependency}_LIBRARIES})
                if (NOT "${${dependency}_LIBRARY_DIRS}" STREQUAL "")
                    if (NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
                        unset(lib CACHE)
                        find_library(lib NAMES ${library} PATHS ${${dependency}_LIBRARY_DIRS} NO_DEFAULT_PATH)
                        if (lib)
                            set(library ${lib})
                        endif ()
                    endif ()
                endif ()
                list(APPEND targets ${library})
            endforeach ()
        endif ()

        list(APPEND targets ${${dependency}_TARGETS})

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

    return(PROPAGATE
            # dependencies
            PUBLIC_DEPENDENCIES
            PRIVATE_DEPENDENCIES
            INTERFACE_DEPENDENCIES
            DEPENDENCIES
            # dependency targets
            PUBLIC_DEPENDENCY_TARGETS
            PRIVATE_DEPENDENCY_TARGETS
            INTERFACE_DEPENDENCY_TARGETS
            DEPENDENCY_TARGETS
    )
endfunction()
