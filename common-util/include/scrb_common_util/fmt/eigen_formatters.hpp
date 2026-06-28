#pragma once

#include <string>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <fmt/format.h>

// NOLINTBEGIN(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)

template <typename T, int Rows, int Cols>
struct fmt::formatter<Eigen::Matrix<T, Rows, Cols>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::Matrix<T, Rows, Cols>& matrix, FormatContext& ctx) const {
        auto out = ctx.out();

        std::vector<std::vector<std::string>> cells(matrix.rows(), std::vector<std::string>(matrix.cols()));

        for (int row = 0; row < matrix.rows(); row++) {
            for (int col = 0; col < matrix.cols(); col++) {
                auto buf = memory_buffer();
                auto context = FormatContext(appender(buf), format_args());
                formatter<T>::format(matrix(row, col), context);
                cells.at(row).at(col) = std::string(buf.data(), buf.size());
            }
        }

        std::vector<size_t> col_width(matrix.cols(), 0);
        // iterating this in the wrong direction isn't the best, but whatever
        for (int col = 0; col < matrix.cols(); col++) {
            for (int row = 0; row < matrix.rows(); row++) {
                col_width.at(col) = std::max(col_width.at(col), cells.at(row).at(col).size());
            }
        }

        for (int rpw = 0; rpw < matrix.rows(); rpw++) {
            format_to(out, "[");
            for (int col = 0; col < matrix.cols(); col++) {
                if (col != 0)
                    format_to(out, " ");

                format_to(out, "{:>{}}", cells.at(rpw).at(col), col_width.at(col));
            }

            format_to(out, "]");
            if (rpw + 1 < matrix.rows())
                format_to(out, "\n");
        }

        return out;
    }
};

// vector formatter
template <typename T, int N>
struct fmt::formatter<Eigen::Matrix<T, N, 1>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::Matrix<T, N, 1>& vector, FormatContext& ctx) const {
        auto out = ctx.out();

        format_to(out, "(");

        for (int i = 0; i < vector.size(); i++) {
            formatter<T>::format(vector(i), ctx);

            if (i + 1 < vector.size())
                format_to(out, ", ");
        }

        format_to(out, ")");

        return out;
    }
};

// row vector formatter
template <typename T, int N>
struct fmt::formatter<Eigen::Matrix<T, 1, N>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::Matrix<T, 1, N>& vector, FormatContext& ctx) const {
        auto out = ctx.out();

        format_to(out, "[");

        for (int i = 0; i < vector.size(); i++) {
            formatter<T>::format(vector(i), ctx);

            if (i + 1 < vector.size())
                format_to(out, ", ");
        }

        format_to(out, "]");

        return out;
    }
};

template <typename T>
struct fmt::formatter<Eigen::Quaternion<T>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::Quaternion<T>& quaternion, FormatContext& ctx) const {
        auto out = ctx.out();

        format_to(out, "(");
        formatter<T>::format(quaternion.w(), ctx);
        format_to(out, ", ");
        formatter<T>::format(quaternion.x(), ctx);
        format_to(out, ", ");
        formatter<T>::format(quaternion.y(), ctx);
        format_to(out, ", ");
        formatter<T>::format(quaternion.z(), ctx);
        format_to(out, ")");

        return out;
    }
};

template <typename T, int Dim, int Mode, int Options>
struct fmt::formatter<Eigen::Transform<T, Dim, Mode, Options>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::Transform<T, Dim, Mode, Options>& transform, FormatContext& ctx) const {
        auto out = ctx.out();
        const auto& matrix = transform.matrix();

        std::array<std::array<std::string, Dim + 1>, Dim + 1> cells{};

        for (int row = 0; row < Dim + 1; row++) {
            for (int col = 0; col < Dim + 1; col++) {
                auto buf = memory_buffer();
                auto context = FormatContext(appender(buf), format_args());
                formatter<T>::format(matrix(row, col), context);
                cells[row][col] = std::string(buf.data(), buf.size());
            }
        }

        std::array<size_t, Dim + 1> col_width{};

        for (int col = 0; col < Dim + 1; col++) {
            for (int row = 0; row < Dim + 1; row++) {
                col_width[col] = std::max(col_width[col], cells[row][col].size());
            }
        }

        for (int row = 0; row < Dim + 1; row++) {
            format_to(out, "[");

            for (int col = 0; col < Dim; col++) {
                format_to(out, "{:>{}} ", cells[row][col], col_width[col]);
            }

            format_to(out, "| {:>{}}]", cells[row][Dim], col_width[Dim]);

            if (row + 1 < Dim + 1)
                format_to(out, "\n");
        }

        return out;
    }
};

template <typename T, int Dim>
struct fmt::formatter<Eigen::AlignedBox<T, Dim>> : formatter<Eigen::Matrix<T, Dim, 1>> {
    template <typename FormatContext>
    auto format(const Eigen::AlignedBox<T, Dim> box, FormatContext& ctx) const {
        auto out = ctx.out();

        if (!box.isEmpty()) {
            format_to(out, "[ ");
            formatter<Eigen::Matrix<T, Dim, 1>>::format(box.min(), ctx);
            format_to(out, ", ");
            formatter<Eigen::Matrix<T, Dim, 1>>::format(box.max(), ctx);
            format_to(out, " ]");
        } else {
            format_to(out, "[ empty ]");
        }

        return out;
    }
};

template <typename T>
struct fmt::formatter<Eigen::AngleAxis<T>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::AngleAxis<T> angle_axis, FormatContext& ctx) const {
        auto out = ctx.out();

        auto angle = angle_axis.angle();
        auto& axis = angle_axis.axis();

        formatter<T>::format(angle, ctx);

        format_to(out, " * (");

        for (int i = 0; i < axis.size(); i++) {
            formatter<T>::format(axis(i), ctx);

            if (i + 1 < axis.size())
                format_to(out, ", ");
        }

        format_to(out, ")");

        return out;
    }
};

template <typename T, int Options, typename Index>
struct fmt::formatter<Eigen::SparseMatrix<T, Options, Index>> : formatter<T> {
    template <typename FormatContext>
    auto format(const Eigen::SparseMatrix<T, Options, Index>& matrix, FormatContext& ctx) const {
        auto out = ctx.out();

        std::vector<std::vector<std::string>> cells(matrix.rows(), std::vector<std::string>(matrix.cols()));
        for (int k = 0; k < matrix.outerSize(); ++k) {
            for (typename Eigen::SparseMatrix<T, Options, Index>::InnerIterator it(matrix, k); it; ++it) {
                auto buf = memory_buffer();
                auto context = FormatContext(appender(buf), format_args());
                formatter<T>::format(it.value(), context);
                cells[it.row()][it.col()] = std::string(buf.data(), buf.size());
            }
        }

        std::vector<size_t> col_width(matrix.cols(), 1);

        for (int col = 0; col < matrix.cols(); ++col) {
            for (int row = 0; row < matrix.rows(); ++row) {
                col_width.at(col) = std::max(col_width.at(col), cells.at(row).at(col).size());
            }
        }

        for (int row = 0; row < matrix.rows(); ++row) {
            format_to(out, "[");

            for (int col = 0; col < matrix.cols(); ++col) {
                if (col != 0)
                    format_to(out, " ");

                format_to(out, "{:>{}}", cells.at(row).at(col), col_width.at(col));
            }

            format_to(out, "]");

            if (row + 1 < matrix.rows())
                format_to(out, "\n");
        }

        return out;
    }
};

// NOLINTEND(*-pro-bounds-avoid-unchecked-container-access, *-pro-bounds-constant-array-index)

