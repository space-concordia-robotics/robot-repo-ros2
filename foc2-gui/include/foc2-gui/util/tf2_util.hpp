#pragma once

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
    inline Quaternion fromMsg(const geometry_msgs::msg::Quaternion& msg) {
        Quaternion result;
        fromMsg(msg, result);

        return result;
    }
}
