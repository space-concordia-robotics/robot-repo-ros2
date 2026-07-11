function(CPMAddFmt)
    CPMAddPackage(
            NAME fmt
            GIT_REPOSITORY "https://github.com/fmtlib/fmt.git"
            VERSION "12.2.0"
            GIT_TAG "12.2.0" # fmt does not prefix tags with v
            GIT_PROGRESS TRUE
    )
endfunction()

function(CPMAddMagicEnum)
    CPMAddPackage(
            NAME magic_enum
            GIT_REPOSITORY "https://github.com/Neargye/magic_enum.git"
            VERSION "0.9.8"
            GIT_PROGRESS TRUE
    )
endfunction()
