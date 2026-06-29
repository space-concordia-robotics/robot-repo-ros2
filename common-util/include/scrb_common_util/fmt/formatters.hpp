#pragma once

// mmmm, I looovveee funny macros

#if __has_include(<Eigen/Core>)
#include "scrb_common_util/fmt/eigen_formatters.hpp"
#endif

#if __has_include(<opencv2/core.hpp>)
#include "scrb_common_util/fmt/opencv_formatters.hpp"
#endif

#if __has_include(<tf2/convert.hpp>)
#include "scrb_common_util/fmt/tf2_formatters.hpp"
#endif

#include "scrb_common_util/fmt/ros_formatters.hpp"
