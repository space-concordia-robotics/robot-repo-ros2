// Copyright 2022-2025 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <chrono>
#include <mutex>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/camera_common.hpp>
#include <magic_enum/magic_enum.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "ros_aruco_opencv/board_loader.hpp"
#include "ros_aruco_opencv/detector.hpp"
#include "ros_aruco_opencv/parameters.hpp"
#include "ros_aruco_opencv/utils.hpp"
#include "ros_aruco_opencv_msgs/msg/aruco_detections.hpp"

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace ros_aruco_opencv {
    class ArucoTracker : public rclcpp_lifecycle::LifecycleNode {
        ros2_fmt_logger::Logger logger;

        // Parameters
        CoreParams params_;
        DetectorParams detector_params_;
        cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;
        bool transform_poses_;

        // ROS
        OnSetParametersCallbackHandle::SharedPtr on_set_parameter_callback_handle_;
        PostSetParametersCallbackHandle::SharedPtr post_set_parameter_callback_handle_;
        rclcpp_lifecycle::LifecyclePublisher<ros_aruco_opencv_msgs::msg::ArucoDetections>::SharedPtr detection_pub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_sub_;
        rclcpp::Time last_msg_stamp_;
        bool cam_info_retrieved_ = false;
        rclcpp::Time callback_start_time_;

        // Aruco
        std::vector<std::pair<std::string, cv::Ptr<cv::aruco::Board>>> boards_;
        std::unique_ptr<ArucoDetector> detector_;

        // Tf2
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    public:
        explicit ArucoTracker(const rclcpp::NodeOptions& options) : LifecycleNode("aruco_tracker", options), logger(get_logger()) {
            declare_parameters();
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
            logger.info("Configuring");

#if CV_VERSION_MAJOR > 4 || CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7
            aruco_parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
#else
            aruco_parameters_ = cv::aruco::DetectorParameters::create();
#endif

            retrieve_parameters();

            if (!ARUCO_DICT_MAP.contains(params_.marker_dict)) {
                logger.error("Unsupported dictionary name: {}", params_.marker_dict);
                return CallbackReturn::FAILURE;
            }

            detector_ = std::make_unique<ArucoDetector>(get_logger().get_child("ArucoDetector"));
            detector_->set_dictionary(params_.marker_dict);
            detector_->set_detector_parameters(detector_params_);
            detector_->set_aruco_parameters(aruco_parameters_);

            if (!params_.board_descriptions_path.empty()) {
                load_boards();
            }
            detector_->set_boards(boards_);

            if (params_.publish_tf) {
                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
            }

            detection_pub_ = create_publisher<ros_aruco_opencv_msgs::msg::ArucoDetections>("aruco_detections", 5);
            debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", 5);

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
            logger.info("Activating");

            if (transform_poses_) {
                tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            }

            LifecycleNode::on_activate(state);

            detection_pub_->on_activate();
            debug_pub_->on_activate();

            on_set_parameter_callback_handle_ = add_on_set_parameters_callback(
                std::bind(&ArucoTracker::callback_on_set_parameters, this, std::placeholders::_1)
            );
            post_set_parameter_callback_handle_ = add_post_set_parameters_callback(
                std::bind(&ArucoTracker::callback_post_set_parameters, this, std::placeholders::_1)
            );

            logger.info("Waiting for first camera info...");

            cam_info_retrieved_ = false;

            const std::string image_topic = rclcpp::expand_topic_or_service_name(params_.cam_base_topic, this->get_name(), this->get_namespace());
            const std::string cam_info_topic = image_transport::getCameraInfoTopic(image_topic);

            cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                cam_info_topic, 1,
                std::bind(&ArucoTracker::callback_camera_info, this, std::placeholders::_1)
            );

            rmw_qos_profile_t image_sub_qos = rmw_qos_profile_default;
            image_sub_qos.reliability = static_cast<rmw_qos_reliability_policy_t>(params_.qos_rel);
            image_sub_qos.durability = static_cast<rmw_qos_durability_policy_t>(params_.qos_dur);
            image_sub_qos.depth = params_.qos_depth;

            const auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_sub_qos), image_sub_qos);

            if (params_.image_sub_compressed) {
                compressed_img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                    fmt::format("{}/{}", image_topic, "compressed"), qos,
                    std::bind(&ArucoTracker::callback_compressed_image, this, std::placeholders::_1)
                );
            } else {
                img_sub_ = create_subscription<sensor_msgs::msg::Image>(
                    image_topic, qos,
                    std::bind(&ArucoTracker::callback_image, this, std::placeholders::_1)
                );
            }

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override {
            logger.info("Deactivating");

            on_set_parameter_callback_handle_.reset();
            post_set_parameter_callback_handle_.reset();
            cam_info_sub_.reset();
            img_sub_.reset();
            compressed_img_sub_.reset();
            tf_listener_.reset();
            tf_buffer_.reset();

            detection_pub_->on_deactivate();
            debug_pub_->on_deactivate();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
            logger.info("Cleaning up");

            tf_broadcaster_.reset();
            aruco_parameters_.reset();
            detector_.reset();
            detection_pub_.reset();
            debug_pub_.reset();
            boards_.clear();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override {
            logger.info("Shutting down");

            on_set_parameter_callback_handle_.reset();
            post_set_parameter_callback_handle_.reset();
            cam_info_sub_.reset();
            img_sub_.reset();
            compressed_img_sub_.reset();
            tf_listener_.reset();
            tf_buffer_.reset();
            tf_broadcaster_.reset();
            aruco_parameters_.reset();
            detector_.reset();
            detection_pub_.reset();
            debug_pub_.reset();
            boards_.clear();

            return CallbackReturn::SUCCESS;
        }

    protected:
        void declare_parameters() {
            declare_all_parameters(*this);
        }

        void retrieve_parameters() {
            params_ = retrieve_core_parameters(*this);
            detector_params_ = retrieve_detector_parameters(*this);

            logger.info("Assume images are rectified: {}", params_.image_is_rectified ? "YES" : "NO");
            if (params_.output_frame.empty()) {
                logger.info("Marker detections will be published in the camera frame");
                transform_poses_ = false;
            } else {
                logger.info("Marker detections will be transformed to \'{}\' frame", params_.output_frame);
                transform_poses_ = true;
            }
            logger.info("TF publishing is {}", params_.publish_tf ? "enabled" : "disabled");
            logger.info("Marker size: {} meters", detector_params_.marker_size);
            logger.info("Pose selector strategy: {}", magic_enum::enum_name(detector_params_.pose_selector.strategy));
            logger.info("Aruco Parameters:");

            retrieve_aruco_parameters(*this, aruco_parameters_, true);
        }

        rcl_interfaces::msg::SetParametersResult callback_on_set_parameters(const std::vector<rclcpp::Parameter>& parameters) const {
            auto result = validate_core_parameters(parameters);
            if (!result.successful) {
                logger.error("Error while validating parameters: {}", result.reason);
            }
            result = validate_detector_parameters(parameters);
            if (!result.successful) {
                logger.error("Error while validating parameters: {}", result.reason);
            }
            return result;
        }

        void callback_post_set_parameters(const std::vector<rclcpp::Parameter>& parameters) {
            update_dynamic_parameters(*this, parameters, detector_params_, aruco_parameters_);

            detector_->set_detector_parameters(detector_params_);
            detector_->set_aruco_parameters(aruco_parameters_);
        }

        void load_boards() {
            logger.info("Trying to load board descriptions from {}", params_.board_descriptions_path);

            std::vector<std::pair<std::string, cv::Ptr<cv::aruco::Board>>> loaded;
            if (std::string err; !BoardLoader::load_from_file(params_.board_descriptions_path, detector_->get_dictionary(),
                                                              loaded, err)) {
                logger.error("Error loading board from file: {}", err);
                return;
            }
            boards_ = std::move(loaded);
            for (const auto& [name, _] : boards_) {
                logger.info("Successfully loaded configuration for board '{}'", name);
            }
        }

        void callback_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info) {
            detector_->update_camera_info(*cam_info, params_.image_is_rectified);

            if (!cam_info_retrieved_) {
                logger.info("First camera info retrieved.");
                cam_info_retrieved_ = true;
            }
        }

        void callback_compressed_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& img_msg) {
            if (!should_process_img_msg(img_msg)) {
                return;
            }

            const auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
            process_image(cv_ptr);
        }

        void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
            if (!should_process_img_msg(img_msg)) {
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            if (sensor_msgs::image_encodings::hasAlpha(img_msg->encoding)) {
                cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
            } else {
                cv_ptr = cv_bridge::toCvShare(img_msg);
            }
            process_image(cv_ptr);
        }

        template <typename ImgMsgT>
        bool should_process_img_msg(ImgMsgT img_msg) {
            logger.debug("Image message address [SUBSCRIBE]:\t{}", reinterpret_cast<const void*>(img_msg.get()));

            if (!cam_info_retrieved_) {
                logger.debug("Camera info not retrieved yet. Ignoring image...");
                return false;
            }

            if (img_msg->header.stamp == last_msg_stamp_) {
                logger.debug("The new image has the same timestamp as the previous one (duplicate frame?). Ignoring...");
                return false;
            }

            last_msg_stamp_ = img_msg->header.stamp;

            // We're ready to go, remember the current time to measure callback performance.
            callback_start_time_ = get_clock()->now();

            return true;
        }

        void process_image(const cv_bridge::CvImageConstPtr& cv_ptr) {
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            std::vector<std::vector<cv::Point2f>> rejected_corners;
            detector_->detect(cv_ptr->image, marker_ids, marker_corners, rejected_corners);

            std::vector<cv::Vec3d> rotations_final, translations_final;

            ros_aruco_opencv_msgs::msg::ArucoDetections detections;
            detections.header.frame_id = cv_ptr->header.frame_id;
            detections.header.stamp = cv_ptr->header.stamp;

            detector_->estimate_marker_poses(marker_ids, marker_corners, detections.markers, rotations_final, translations_final);

            for (auto& marker : detections.markers) {
                marker.header.frame_id = cv_ptr->header.frame_id;
                marker.header.stamp = cv_ptr->header.stamp;
            }

            detector_->estimate_board_poses(marker_ids, marker_corners, detections.boards, rotations_final, translations_final);

            detections.rejected_markers.resize(rejected_corners.size());
            for (auto i = 0u; i < rejected_corners.size(); ++i) {
                const auto& corners = rejected_corners[i];

                for (auto j = 0u; j < corners.size(); ++j) {
                    const auto& point = corners.at(j);
                    auto& corner = detections.rejected_markers[i].corners[j];
                    corner.x = point.x;
                    corner.y = point.y;
                    corner.z = 0;
                }
            }

            if (transform_poses_ && (detections.markers.size() > 0 || detections.boards.size() > 0)) {
                detections.header.frame_id = params_.output_frame;
                geometry_msgs::msg::TransformStamped cam_to_output;
                // Retrieve camera -> output_frame transform
                try {
                    cam_to_output = tf_buffer_->lookupTransform(
                        params_.output_frame,
                        cv_ptr->header.frame_id,
                        cv_ptr->header.stamp,
                        rclcpp::Duration::from_seconds(1.0)
                    );
                } catch (tf2::TransformException& ex) {
                    logger.error("Error while looking up transform from camera to output: {}", ex.what());
                    return;
                }
                for (auto& marker_pose : detections.markers) {
                    tf2::doTransform(marker_pose.pose, marker_pose.pose, cam_to_output);
                }
                for (auto& board_pose : detections.boards) {
                    tf2::doTransform(board_pose.pose, board_pose.pose, cam_to_output);
                }
            }

            if (params_.publish_tf && detections.markers.size() > 0) {
                std::vector<geometry_msgs::msg::TransformStamped> transforms;
                for (auto& marker_pose : detections.markers) {
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header.stamp = detections.header.stamp;
                    transform.header.frame_id = detections.header.frame_id;
                    transform.child_frame_id = fmt::format("marker_{}", marker_pose.id);
                    tf2::Transform tf_transform;
                    tf2::fromMsg(marker_pose.pose, tf_transform);
                    transform.transform = tf2::toMsg(tf_transform);
                    transforms.push_back(transform);
                }
                for (auto& board_pose : detections.boards) {
                    geometry_msgs::msg::TransformStamped transform;
                    transform.header.stamp = detections.header.stamp;
                    transform.header.frame_id = detections.header.frame_id;
                    transform.child_frame_id = fmt::format("board_{}", board_pose.board_name);
                    tf2::Transform tf_transform;
                    tf2::fromMsg(board_pose.pose, tf_transform);
                    transform.transform = tf2::toMsg(tf_transform);
                    transforms.push_back(transform);
                }
                tf_broadcaster_->sendTransform(transforms);
            }

            detection_pub_->publish(detections);

            if (debug_pub_->get_subscription_count() > 0) {
                auto debug_cv_ptr = std::make_shared<cv_bridge::CvImage>();
                debug_cv_ptr->header = cv_ptr->header;
                debug_cv_ptr->encoding = cv_ptr->encoding;
                debug_cv_ptr->image = cv_ptr->image.clone();
                cv::aruco::drawDetectedMarkers(debug_cv_ptr->image, marker_corners, marker_ids);

                // rejected markers
                cv::aruco::drawDetectedMarkers(debug_cv_ptr->image, rejected_corners, cv::noArray(), cv::Scalar(100, 0, 255));

                {
                    cv::Mat camera_matrix, distortion_coeffs;
                    detector_->get_intrinsics(camera_matrix, distortion_coeffs);

                    for (auto i = 0u; i < rotations_final.size(); i++) {
                        cv::drawFrameAxes(
                            debug_cv_ptr->image,
                            camera_matrix, distortion_coeffs,
                            rotations_final[i],
                            translations_final[i],
                            0.2, 3
                        );
                    }
                }

                auto debug_img = std::make_unique<sensor_msgs::msg::Image>();
                debug_cv_ptr->toImageMsg(*debug_img);
                debug_pub_->publish(std::move(debug_img));
            }

            auto callback_end_time = get_clock()->now();
            double whole_callback_duration = (callback_end_time - callback_start_time_).seconds();
            double image_send_duration = (callback_start_time_ - cv_ptr->header.stamp).seconds();

            logger.debug(
                "Image callback completed. The callback started {:.4f}s after the image frame was grabbed and completed its execution in {:.4f} s.",
                image_send_duration, whole_callback_duration
            );
        }
    };

    class ArucoTrackerAutostart : public ArucoTracker {
    public:
        explicit ArucoTrackerAutostart(const rclcpp::NodeOptions& options) : ArucoTracker(options) {
            if (const auto new_state = configure(); new_state.label() == "inactive") {
                activate();
            }
        }
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_aruco_opencv::ArucoTracker)

RCLCPP_COMPONENTS_REGISTER_NODE(ros_aruco_opencv::ArucoTrackerAutostart)
