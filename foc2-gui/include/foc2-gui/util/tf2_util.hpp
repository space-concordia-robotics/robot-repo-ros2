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

    template <>
    constexpr Eigen::Isometry3d fromMsg(const geometry_msgs::msg::Transform& msg) {
        return Eigen::Translation3d(fromMsg<Eigen::Vector3d>(msg.translation)) * fromMsg<Eigen::Quaterniond>(msg.rotation);
    }

    template <>
    inline Eigen::Isometry3d fromMsg(const geometry_msgs::msg::TransformStamped& msg) {
        return fromMsg<Eigen::Isometry3d>(msg.transform);
    }
}
