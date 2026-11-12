#pragma once

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "foc2-gui/resources.hpp"

static const auto SHARE_DIR = std::filesystem::path(ament_index_cpp::get_package_share_directory(FOC2_PACKAGE_NAME));

static const auto RESOURCES_DIR = SHARE_DIR / "resources";
