#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tf2 {
    /**
     * Function that converts from a ROS message type to another type. It has to be
     * implemented by each data type in tf2_* (except ROS messages) as it is used
     * in the "convert" function.
     * @param msg a ROS message to convert from
     * @return the converted object
     */
    template <typename Converted, typename Msg>
    Converted fromMsg(const Msg& msg) {
        Converted result;
        tf2::fromMsg(msg, result);
        return result;
    }

    /**
     * Function that converts from one type to a ROS message type. It has to be
     * implemented by each data type in tf2_* (except ROS messages) as it is
     * used in the "convert" function.
     * @param a an object of whatever type
     * @return the conversion as a ROS message
     */
    template <typename A, typename B>
    B toMsg(const A& a) {
        return toMsg(a);
    }

    /**
     * Function that converts any type to any type (messages or not).
     * Matching toMsg and from Msg conversion functions need to exist.
     * If they don't exist or do not apply (for example, if your two
     * classes are ROS messages), just write a specialization of the function.
     * @param input an object to convert from
     */
    template <typename Converted, typename Original>
    Converted convert(const Original& input) {
        Converted result;
        tf2::convert(input, result);
        return result;
    }

    template <>
    inline Eigen::Isometry3d fromMsg(const geometry_msgs::msg::Transform& msg) {
        return Eigen::Translation3d(fromMsg<Eigen::Vector3d>(msg.translation)) * fromMsg<Eigen::Quaterniond>(msg.rotation);
    }

    template <>
    inline Eigen::Isometry3d fromMsg(const geometry_msgs::msg::TransformStamped& msg) {
        return fromMsg<Eigen::Isometry3d>(msg.transform);
    }

    template <>
    inline Eigen::Matrix3d convert(const Matrix3x3& input) {
        Eigen::Matrix<double, 3, 3> matrix;
        for (int row = 0; row < 3; ++row) {
            const auto& matrix_row = input.getRow(row);
            matrix(row, 0) = matrix_row.x();
            matrix(row, 1) = matrix_row.y();
            matrix(row, 2) = matrix_row.z();
        }

        return matrix;
    }
}
