#pragma once

#include <Eigen/Core>
#include <imgui.h>

#include "scrb_common_util/tf2_util.hpp"

namespace tf2 {
    template <>
    inline Eigen::Vector2d convert(const ImVec2& input) {
        return {input.x, input.y};
    }

    template <>
    inline Eigen::Vector4d convert(const ImVec4& input) {
        return {input.x, input.y, input.z, input.w};
    }

    template <>
    inline ImVec2 convert(const Eigen::Vector2d& input) {
        const auto vec = input.cast<float>();
        return {vec.x(), vec.y()};
    }

    template <>
    inline ImVec4 convert(const Eigen::Vector4d& input) {
        const auto vec = input.cast<float>();
        return {vec.x(), vec.y(), vec.z(), vec.w()};
    }
}

// region ImGui internals

struct ImVec2i;
struct ImRect;

namespace tf2 {
    template <>
    inline Eigen::Vector2i convert(const ImVec2i& input) {
        return {input.x, input.y};
    }

    template <>
    inline ImVec2i convert(const Eigen::Vector2i& input) {
        return {input.x(), input.y()};
    }

    template <>
    inline Eigen::AlignedBox2d convert(const ImRect& input) {
        return {
            Eigen::Vector2d(input.Min.x, input.Min.y),
            Eigen::Vector2d(input.Max.x, input.Max.y)
        };
    }

    template <>
    inline ImRect convert(const Eigen::AlignedBox2d& input) {
        const auto box = input.cast<float>();
        return {
            box.min().x(), box.min().y(),
            box.max().x(), box.max().y()
        };
    }
}

// endregion
