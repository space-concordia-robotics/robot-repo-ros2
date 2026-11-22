# Common Utilities

This package contains some common utilities shared across several other packages.

## String Parsing

This package provides utility functions for string parsing.
Currently, all of the functions throw on invalid input, however this is subject to change.

### Floats

- `scrb::common_util::parse_double<Stringish S>(const S& input) -> double`: parses a double
- `scrb::common_util::parse_float<Stringish S>(const S& input) -> float`: parses a float

### Integers

- `scrb::common_util::parse_int8<Stringish S>(S& input, int base = 10) -> int8_t` parses an int8
- `scrb::common_util::parse_uint8<Stringish S>(S& input, int base = 10) -> uint8_t`: parses a uint8
- `scrb::common_util::parse_int16<Stringish S>(S& input, int base = 10) -> int16_t`: parses an int16
- `scrb::common_util::parse_uint16<Stringish S>(S& input, int base = 10) -> uint16_t`: parses a uint16
- `scrb::common_util::parse_int32<Stringish S>(S& input, int base = 10) -> int32_t`: parses an int32
- `scrb::common_util::parse_uint32<Stringish S>(S& input, int base = 10) -> uint32_t`: parses a uint32
- `scrb::common_util::parse_int64<Stringish S>(S& input, int base = 10) -> int64_t`: parses an int64
- `scrb::common_util::parse_uint64<Stringish S>(S& input, int base = 10) -> uint64_t`: parses a uint64

### Other

- `scrb::common_util::parse_bool<Stringish S>(const S& input) -> bool`: parses a boolean
- `scrb::common_util::parse_duration<Stringish S>(const S& input) -> std::chrono::nanoseconds`: parses a duration
- `scrb::common_util::parse_array<typename T>(const std::string& array_string) -> std::vector<T>`: parses an array of `T`.
  `T` must be a boolean, floating-point, integral, or string type.

## tf2 Conversion Utilities

The following conversion utilities are provided for tf2:

- `tf2::fromMsg<typename Converted, typename Msg>(const Msg& msg) -> Converted`: overload of `tf2::fromMsg` which returns the converted message,
  rather than taking the result as a reference parameter.
- `tf2::toMsg<typename A, typename B>(const A& a) -> B`: template specialization for ROS types that provide `tf2::toMsg` overloads that *aren't* templates.
- `tf2::convert<typename Converted, typename Original>(const Original& input) -> Converted`: overload of `tf2::convert` which returns the converted type,
  rather than taking the result as a reference parameter.
- `tf2::fromMsg<>(const geometry_msgs::msg::Transform& msg) -> Eigen::Isometry3d`: template specialization for converting a
  `geometry_msgs::msg::Transform` to a `Eigen::Isometry3d`.
- `tf2::fromMsg<>(const geometry_msgs::msg::TransformStamped& msg) -> Eigen::Isometry3d`: template specialization for converting a
  `geometry_msgs::msg::Transform` to a `Eigen::Isometry3d`.
- `tf2::convert<>(const tf2::Matrix3x3& input) -> Eigen::Matrix3d`: template specialization for converting a `tf2::Matrix3x3` to an `Eigen::Matrix3d`.

The template specializations should be expanded as missing ones are encountered.

## Formatters

This package provides several [fmt](https://fmt.dev/) formatters.

### Eigen Formatters

Formatters are provided for the following Eigen types:

- `Eigen::Matrix<T, Rows, Cols>`
- `Eigen::Matrix<T, N, 1>` (`Eigen::Vector<T, N>`)
- `Eigen::Matrix<T, 1, N>` (`Eigen::RowVectorVector<T, N>`)
- `Eigen::Quaternion<T>`
- `Eigen::Transform<T, Dim, Mode, Options>` (includes `Eigen::Isometry*`, `Eigen::Affine*`, `Eigen::AffineCompact`, `Eigen::Projective*`)
- `Eigen::AlignedBox<T, Dim>`
- `Eigen::AngleAxis<T>`
- `Eigen::SparseMatrix<T, Options, Index>`

### OpenCV Formatters

Formatters are provided for the following OpenCV types (delegates to Eigen formatters):

- `cv::Vec<T, N>`
- `cv::Matx<T, N, M>`

### ROS Formatters

Formatters are provided for the following ROS types:

- `rclcpp::Time`
- `rclcpp::Duration`
- `builtin_interfaces::msg::Time`
- `builtin_interfaces::msg::Duration`
- `rclcpp::Rate` & `rclcpp::WallRate`

### tf2 Formatters

Formatters are provided for the following tf2 types (delegates to Eigen formatters):

- `tf2::Vector3`
- `tf2::Quaternion`
- `tf2::Matrix3x3`
- `tf2::Transform`
