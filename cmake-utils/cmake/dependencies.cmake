# this is a macro so that the caller inherits all the variables
macro(find_dependencies)
    set(found_packages "")

    set(dependencies "${ARGN}")
    foreach (dependency IN LISTS dependencies)
        separate_arguments(dependency)

        list(POP_FRONT dependency name)
        find_package(${name} REQUIRED ${dependency})
        list(APPEND found_packages "${name}")
    endforeach ()

    # Return only the package names
    set(DEPENDENCIES "${found_packages}")

    set(DEPENDENCY_TARGETS "")
    foreach (dependency IN LISTS DEPENDENCIES)
        # I think this checks out? I yoinked it from ament_target_dependencies()

        set(use_modern_cmake FALSE)
        if (NOT "${${dependency}_TARGETS}" STREQUAL "")
            foreach (_target ${${dependency}_TARGETS})
                # only use actual targets
                # in case a package uses this variable for other content
                if (TARGET "${_target}")
                    list_append_unique(DEPENDENCY_TARGETS ${_target})
                    set(use_modern_cmake TRUE)
                endif ()
            endforeach ()
        endif ()
        if (NOT use_modern_cmake)
            # otherwise use the classic CMake variables
            message(WARNING "${${dependency}_LIBRARIES}")
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
                list(APPEND DEPENDENCY_TARGETS ${library})
            endforeach ()
        endif ()

        list(APPEND DEPENDENCY_TARGETS ${${dependency}_TARGETS})
    endforeach ()

    message(WARNING "dependencies: ${DEPENDENCIES}, dependency targets: ${DEPENDENCY_TARGETS}")

    # clean up variables
    unset(dependencies)
    unset(dependency)
    unset(found_packages)
    unset(name)
endmacro()
