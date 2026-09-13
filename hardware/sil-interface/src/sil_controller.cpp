#include "sil_interface/sil_controller.hpp"

// TODO 2026-05-13 (Will Free): tbh, I'm not sure if this is the best implementation. I just kinda bodged this together.
namespace sil_interface {
    template <typename F, typename I>
    constexpr F normalize(I value) noexcept {
        static_assert(std::is_integral_v<I>, "I must be an integral type");
        static_assert(std::is_unsigned_v<I>, "I must be an unsigned integral type"); // TODO 2026-05-13 (Will Free): support signed types
        static_assert(std::is_floating_point_v<F>, "T must be floating point type");

        return static_cast<F>(value) / std::numeric_limits<I>::max();
    }

    using SetSILStatus = rover_msgs::srv::SetSILStatus;

    void resetStatus(const std::shared_ptr<SILStatus>& status) {
        status->red = 0;
        status->blue = 0;
        status->green = 0;
        status->brightness = 0;
    }

    SILController::SILController() = default;

    controller_interface::CallbackReturn SILController::on_init() {
        logger = std::make_shared<ros2_fmt_logger::Logger>(get_node()->get_logger());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SILController::command_interface_configuration() const {
        return {
            .type  = controller_interface::interface_configuration_type::INDIVIDUAL,
            .names = {
                "sil/red",
                "sil/green",
                "sil/blue",
                "sil/brightness",
            }
        };
    }

    controller_interface::InterfaceConfiguration SILController::state_interface_configuration() const {
        return {
            .type  = controller_interface::interface_configuration_type::NONE,
            .names = {}
        };
    }

    controller_interface::CallbackReturn SILController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // TODO 2026-05-13 (Will Free): any parameters

        const auto& node = get_node();

        sil_status_service = node->create_service<SetSILStatus>(
            "~/set_status",
            [this](const SetSILStatus::Request::SharedPtr& request, const SetSILStatus::Response::SharedPtr& response) {
                handleSetSILStatus(request, response);
            }
        );

        const auto msg = std::make_shared<SILStatus>();
        resetStatus(msg);
        status_ref.writeFromNonRT(msg);

        return ControllerInterface::on_configure(previous_state);
    }

    controller_interface::CallbackReturn SILController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        resetStatus(*status_ref.readFromRT());

        return ControllerInterface::on_activate(previous_state);
    }

    controller_interface::CallbackReturn SILController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        auto success = true;
        for (auto& interface : command_interfaces_) {
            // ReSharper disable once CppDFAUnusedValue
            success &= interface.set_value(0);
        }

        if (!success)
            return CallbackReturn::FAILURE;

        return ControllerInterface::on_deactivate(previous_state);
    }

    controller_interface::return_type SILController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        const auto current_ref = status_ref.readFromRT();

        if (current_ref == nullptr)
            return controller_interface::return_type::ERROR;

        const auto status = *current_ref;

        auto success = true;
        // TODO 2026-05-13 (Will Free): make the indexes for this not hardcoded
        success &= command_interfaces_.at(0).set_value(normalize<double>(status->red));
        success &= command_interfaces_.at(1).set_value(normalize<double>(status->green));
        success &= command_interfaces_.at(2).set_value(normalize<double>(status->blue));
        success &= command_interfaces_.at(3).set_value(normalize<double>(status->brightness));

        if (!success)
            return controller_interface::return_type::ERROR;

        return controller_interface::return_type::OK;
    }

    void SILController::handleSetSILStatus(
        const SetSILStatus::Request::SharedPtr& request,
        const SetSILStatus::Response::SharedPtr& /*response*/
    ) {
        const auto status = std::make_shared<SILStatus>(
            request->r,
            request->g,
            request->b,
            request->brightness
        );

        status_ref.writeFromNonRT(status);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sil_interface::SILController, controller_interface::ControllerInterface)
