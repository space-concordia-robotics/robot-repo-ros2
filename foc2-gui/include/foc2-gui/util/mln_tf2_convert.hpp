#pragma once

#include <cstdint>
#include <imgui.h>
#include <Eigen/Core>
#include <mbgl/util/geo.hpp>
#include <mbgl/util/size.hpp>

#include "scrb_common_util/tf2_util.hpp"

namespace tf2 {
    /// only valid for positive vectors
    template <>
    inline mln::Size convert(const ImVec2& input) {
        return {static_cast<uint32_t>(input.x), static_cast<uint32_t>(input.y)};
    }

    /// only valid for positive vectors
    template <>
    inline mln::Size convert(const Eigen::Vector2i& input) {
        return {static_cast<uint32_t>(input.x()), static_cast<uint32_t>(input.y())};
    }

    /// only valid for positive vectors
    template <>
    inline mln::Size convert(const Eigen::Vector2d& input) {
        return {static_cast<uint32_t>(input.x()), static_cast<uint32_t>(input.y())};
    }

    template <>
    inline mln::ScreenCoordinate convert(const ImVec2& input) {
        return {input.x, input.y};
    }

    template <>
    inline mln::ScreenCoordinate convert(const Eigen::Vector2d& input) {
        return {input.x(), input.y()};
    }
}
