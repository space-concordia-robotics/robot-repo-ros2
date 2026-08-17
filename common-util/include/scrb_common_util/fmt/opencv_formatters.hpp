#pragma once

#include <fmt/format.h>
#include <opencv2/core/eigen.hpp>

#include "scrb_common_util/fmt/eigen_formatters.hpp"

// NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)

template <typename T, int N>
struct fmt::formatter<cv::Vec<T, N>> : formatter<Eigen::Vector<T, N>> {
    template <typename FormatContext>
    auto format(const cv::Vec<T, N>& vector, FormatContext& ctx) const {
        auto out = ctx.out();

        Eigen::Vector<T, N> vec;
        for (int i = 0; i < N; i++) {
            vec[i] = vector[i];
        }

        return formatter<Eigen::Vector<T, N>>::format(vec, ctx);
    }
};

template <typename T, int N, int M>
struct fmt::formatter<cv::Matx<T, N, M>> : formatter<Eigen::Matrix<T, N, M>> {
    template <typename FormatContext>
    auto format(const cv::Matx<T, N, M>& matrix, FormatContext& ctx) const {
        auto out = ctx.out();

        Eigen::Matrix<T, N, M> eigen_matrix;
        cv::cv2eigen(matrix, eigen_matrix);

        return formatter<Eigen::Matrix<T, N, M>>::format(eigen_matrix, ctx);
    }
};

// TODO 2026-06-14 (Will Free): The other opencv types are harder & more annoying to convert to eigen types

// NOLINTEND(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)
