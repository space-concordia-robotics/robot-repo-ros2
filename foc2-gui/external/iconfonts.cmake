include(FetchContent)

FetchContent_Declare(
        iconfontcppheaders
        GIT_REPOSITORY https://github.com/juliettef/IconFontCppHeaders.git
        GIT_TAG 3ee7f3d295ae773c0046db8d7b89b886eb2526de # 2026-05-20
        GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(iconfontcppheaders)

FetchContent_Declare(
        fontawesome
        URL https://github.com/FortAwesome/Font-Awesome/releases/download/7.2.0/fontawesome-free-7.2.0-web.zip
)

FetchContent_MakeAvailable(fontawesome)

set(FONT_AWESOME_SOURCE_PATH "${fontawesome_SOURCE_DIR}/webfonts/fa-solid-900.woff2")

add_library(iconfonts INTERFACE)

target_include_directories(iconfonts INTERFACE ${iconfontcppheaders_SOURCE_DIR})

