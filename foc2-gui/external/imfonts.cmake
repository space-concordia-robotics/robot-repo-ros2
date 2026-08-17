include(FetchContent)

FetchContent_Declare(
        imfonts
        GIT_REPOSITORY https://github.com/gorbatschow/ImFonts.git
        GIT_TAG a57d11888d23bcb9ba52a9902b69c386d26c0e8f
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(imfonts)

add_library(imfonts
        ${imfonts_SOURCE_DIR}/RobotoRegular.cpp
        ${imfonts_SOURCE_DIR}/RobotoBold.cpp
        ${imfonts_SOURCE_DIR}/RobotoItalic.cpp
        ${imfonts_SOURCE_DIR}/RobotoMonoRegular.cpp
        ${imfonts_SOURCE_DIR}/RobotoMonoBold.cpp
        ${imfonts_SOURCE_DIR}/RobotoMonoItalic.cpp
)

target_include_directories(imfonts PUBLIC ${imfonts_SOURCE_DIR})
