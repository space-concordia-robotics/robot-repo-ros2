#include "autonomy/autonomy.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <magic_enum/magic_enum.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include "scrb_common_util/string_parsing.hpp"
#include "autonomy/util.hpp"
#include "autonomy/targets/ar_target.hpp"
#include "autonomy/targets/gps_target.hpp"
#include "autonomy/targets/object_target.hpp"

namespace autonomy {
    AutonomyMissionManager::AutonomyMissionManager()
        : AutonomyMissionManagerInterface("autonomy_mission_manager"),
          co_ctx(std::make_shared<rclcpp_async::CoContext>(*this)), logger(this->get_logger()) {
        parseConfig();

        navsat_topic = this->create_subscription<NavSatFix>(
            "/gps/fix",
            10,
            [this](const NavSatFix::UniquePtr& msg) {
                rover_position.latitude = msg->latitude;
                rover_position.longitude = msg->longitude;
                rover_position.altitude = msg->altitude;
            }
        );

        sil_client = this->create_client<SetSILStatus>("/sil_controller/set_status");

        using namespace std::chrono_literals;
        sil_client->wait_for_service(10s);

        start();
    }


    void AutonomyMissionManager::parseConfig() {
        this->mission_duration = rclcpp::Duration(scrb::common_util::parse_duration(this->declare_parameter<std::string>("mission_duration")));

        const auto gps_config = TargetConfig::GPSConfig{
            .acceptance_radius = this->declare_parameter<double>("target.gps.acceptance_radius"),
            .timeout           = rclcpp::Duration(scrb::common_util::parse_duration(this->declare_parameter<std::string>("target.gps.timeout"))),
            .max_retries       = this->declare_parameter<int>("target.gps.max_retries"),
        };

        const auto ar_config = TargetConfig::ARConfig{
            .topic_name           = this->declare_parameter<std::string>("target.ar.topic_name"),
            .acceptance_radius    = this->declare_parameter<double>("target.ar.acceptance_radius"),
            .timeout              = rclcpp::Duration(scrb::common_util::parse_duration(this->declare_parameter<std::string>("target.ar.timeout"))),
            .max_retries          = this->declare_parameter<int>("target.ar.max_retries"),
            .tag_size             = this->declare_parameter<double>("target.ar.tag_size"),
            .confidence           = this->declare_parameter<double>("target.ar.confidence"),
            .spiral_step          = this->declare_parameter<double>("target.ar.spiral.step"),
            .spiral_radius_factor = this->declare_parameter<double>("target.ar.spiral.radius_factor"),
        };

        const auto object_config = TargetConfig::ObjectConfig{
            .topic_name           = this->declare_parameter<std::string>("target.object.topic_name"),
            .acceptance_radius    = this->declare_parameter<double>("target.object.acceptance_radius"),
            .timeout              = rclcpp::Duration(scrb::common_util::parse_duration(this->declare_parameter<std::string>("target.object.timeout"))),
            .max_retries          = this->declare_parameter<int>("target.object.max_retries"),
            .confidence           = this->declare_parameter<double>("target.object.confidence"),
            .spiral_step          = this->declare_parameter<double>("target.object.spiral.step"),
            .spiral_radius_factor = this->declare_parameter<double>("target.object.spiral.radius_factor"),
        };

        this->target_config = TargetConfig{
            .gps = gps_config,
            .ar = ar_config,
            .object = object_config,
        };

        this->targets = parseTargets();
    }

    std::vector<Target::SharedPtr> AutonomyMissionManager::parseTargets() {
        auto targets = std::vector<Target::SharedPtr>();

        for (const auto target_names = this->declare_parameter<std::vector<std::string>>("targets.names"); const auto& target : target_names) {
            targets.push_back(parseTarget(fmt::format("targets.{}", target), target));
        }

        return targets;
    }

    enum class TargetType {
        GPS,
        AR_TAG,
        OBJECT,
    };

    Target::SharedPtr AutonomyMissionManager::parseTarget(const std::string& prefix, const std::string& target_id) {
        const auto type_str = this->declare_parameter<std::string>(fmt::format("{}.type", prefix));
        const auto type = magic_enum::enum_cast<TargetType>(type_str, magic_enum::case_insensitive);

        if (!type.has_value()) {
            constexpr auto rtsp_camera_type_names = magic_enum::enum_names<TargetType>();
            logger.fatal("Undefined target type '{}', available types are: {}. Check your config file.", type_str, fmt::join(rtsp_camera_type_names, ", "));
            throw std::runtime_error("Undefined target type");
        }

        if (const auto target = type.value(); target == TargetType::GPS) {
            [[maybe_unused]] geographic_msgs::msg::GeoPoint location = parseGeoPoint(fmt::format("{}.location", prefix));

            return GPSTarget::make_shared(*this, target_id, this->target_config.gps, location);
        } else if (target == TargetType::AR_TAG) {
            [[maybe_unused]] geographic_msgs::msg::GeoPoint center = parseGeoPoint(fmt::format("{}.center", prefix));

            [[maybe_unused]] const auto radius = this->declare_parameter<double>(fmt::format("{}.radius", prefix));
            [[maybe_unused]] const auto id = this->declare_parameter<long>(fmt::format("{}.tag_id", prefix));

            return ARTarget::make_shared(*this, target_id, this->target_config.ar, center, radius, id);
        } else if (target == TargetType::OBJECT) {
            [[maybe_unused]] geographic_msgs::msg::GeoPoint center = parseGeoPoint(fmt::format("{}.center", prefix));

            [[maybe_unused]] const auto radius = this->declare_parameter<double>(fmt::format("{}.radius", prefix), 0.0);
            [[maybe_unused]] const auto label = this->declare_parameter<std::string>(fmt::format("{}.label", prefix));

            return ObjectTarget::make_shared(*this, target_id, this->target_config.object, center, radius, label);
        }

        // never happens
        return nullptr;
    }

    geographic_msgs::msg::GeoPoint AutonomyMissionManager::parseGeoPoint(const std::string& prefix) {
        const auto latitude = this->declare_parameter<double>(fmt::format("{}.latitude", prefix));
        const auto longitude = this->declare_parameter<double>(fmt::format("{}.longitude", prefix));
        const auto altitude = this->declare_parameter<double>(fmt::format("{}.altitude", prefix), 0.0);

        geographic_msgs::msg::GeoPoint location;
        location.latitude = latitude;
        location.longitude = longitude;
        location.altitude = altitude;

        return location;
    }

    rclcpp_async::Task<> AutonomyMissionManager::start() {
        setSILColour(0, 0, 255, 255);

        while (true) {
            const auto target = nearestTarget();

            // cannot find any more targets, so we're done
            if (!target)
                break;

            target->setup();

            co_await target->start();

            // set colour back to blue after target
            setSILColour(0, 0, 255, 255);
        }

        co_return;
    }

    Target::SharedPtr AutonomyMissionManager::nearestTarget() const {
        Target::SharedPtr nearest = nullptr;
        auto nearest_distance = std::numeric_limits<double>::max();
        for (const auto& target : targets) {
            if (target->getState() == TargetState::COMPLETED || target->getState() == TargetState::FAILED)
                continue;

            if (const auto distance = util::distance(target->findNearestPoint(rover_position), rover_position); nearest_distance > distance) {
                nearest = target;
                nearest_distance = distance;
            }
        }
        return nearest;
    }

    rclcpp_async::Task<> AutonomyMissionManager::setSILColour(const uint8_t red, const uint8_t green, const uint8_t blue, const uint8_t brightness) {
        using namespace std::chrono_literals;

        const auto sil_request = std::make_shared<SetSILStatus::Request>();
        sil_request->r = red;
        sil_request->g = green;
        sil_request->b = blue;
        sil_request->brightness = brightness;

        co_await co_ctx->send_request<SetSILStatus>(sil_client, sil_request);
    }
}

int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<autonomy::AutonomyMissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
