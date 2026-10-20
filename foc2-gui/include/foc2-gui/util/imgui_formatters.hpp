#pragma once

#include <imgui.h>

#include "foc2-gui/util/imgui_tf2_convert.hpp"
#include "scrb_common_util/fmt/eigen_formatters.hpp"

template <>
struct fmt::formatter<ImVec2> : formatter<Eigen::Vector2d> {
    template <typename FormatContext>
    auto format(const ImVec2& vec, FormatContext& ctx) const {
        auto out = ctx.out();

        formatter<Eigen::Vector2d>::format(tf2::convert<Eigen::Vector2d>(vec), ctx);

        return out;
    }
};

template <>
struct fmt::formatter<ImVec4> : formatter<Eigen::Vector4d> {
    template <typename FormatContext>
    auto format(const ImVec4& vec, FormatContext& ctx) const {
        auto out = ctx.out();

        formatter<Eigen::Vector4d>::format(tf2::convert<Eigen::Vector4d>(vec), ctx);

        return out;
    }
};

// region ImGui internals

struct ImVec2i;

template <>
struct fmt::formatter<ImVec2i> : formatter<Eigen::Vector2i> {
    template <typename FormatContext>
    auto format(const ImVec2i& vec, FormatContext& ctx) const {
        auto out = ctx.out();

        formatter<Eigen::Vector2i>::format(tf2::convert<Eigen::Vector2i>(vec), ctx);

        return out;
    }
};

struct ImRect;

template <>
struct fmt::formatter<ImRect> : formatter<Eigen::AlignedBox2d> {
    template <typename FormatContext>
    auto format(const ImRect& rect, FormatContext& ctx) const {
        auto out = ctx.out();

        formatter<Eigen::AlignedBox2d>::format(tf2::convert<Eigen::AlignedBox2d>(rect), ctx);

        return out;
    }
};

// endregion
