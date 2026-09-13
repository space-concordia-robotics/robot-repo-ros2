#pragma once

#include <fmt/format.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include "scrb_common_util/tf2_util.hpp"
#include "scrb_common_util/fmt/eigen_formatters.hpp"

// NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)

template <>
struct fmt::formatter<tf2::Vector3> : formatter<Eigen::Vector3d> {
    template <typename FormatContext>
    auto format(const tf2::Vector3& vector, FormatContext& ctx) const {
        return formatter<Eigen::Vector3d>::format(tf2::convert<Eigen::Vector3d>(vector), ctx);
    }
};

template <>
struct fmt::formatter<tf2::Quaternion> : formatter<Eigen::Quaterniond> {
    template <typename FormatContext>
    auto format(const tf2::Quaternion& quaternion, FormatContext& ctx) const {
        return formatter<Eigen::Quaterniond>::format(tf2::convert<Eigen::Quaterniond>(quaternion), ctx);
    }
};

template <>
struct fmt::formatter<tf2::Matrix3x3> : formatter<Eigen::Matrix3d> {
    template <typename FormatContext>
    auto format(const tf2::Matrix3x3& matrix, FormatContext& ctx) const {
        return formatter<Eigen::Matrix3d>::format(tf2::convert<Eigen::Matrix3d>(matrix), ctx);
    }
};

template <>
struct fmt::formatter<tf2::Transform> : formatter<Eigen::Isometry3d> {
    template <typename FormatContext>
    auto format(const tf2::Transform& transform, FormatContext& ctx) const {
        const auto trf = tf2::convert<Eigen::Isometry3d>(transform);
        return formatter<Eigen::Isometry3d>::format(trf, ctx);
    }
};

// NOLINTEND(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)
