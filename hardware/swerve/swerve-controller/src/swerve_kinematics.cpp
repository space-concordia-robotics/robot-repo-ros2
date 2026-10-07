#include "swerve_controller/swerve_kinematics.hpp"

#include <cmath>
#include <utility>
#include <angles/angles.h>

namespace swerve_controller {
    SwerveKinematics::SwerveKinematics(const ModulePositions& module_positions)
        : module_positions(module_positions) {
        static const auto rotation_matrix = Eigen::Matrix2d({
            {0.0, 1.0},
            {-1.0, 0.0},
        });

        const auto transformed = module_positions * rotation_matrix;
        module_tangents = transformed.transpose();
    }

    SwerveKinematics::ModuleStates SwerveKinematics::inverse(
        const Eigen::Vector2d& linear_velocity,
        const double angular_velocity,
        const ModuleAngles& current_angles
    ) const {
        ModuleStates states = {};

        const auto wheel_velocities = module_tangents * angular_velocity;

        for (auto i = 0u; i < module_count; ++i) {
            const auto wheel_velocity = wheel_velocities.col(i) + linear_velocity;

            ModuleState state = {
                .speed = wheel_velocity.norm(),
                .angle = std::atan2(wheel_velocity.y(), wheel_velocity.x()),
            };

            if (state.speed <= 1e-9) {
                state.speed = 0.0;
                state.angle = current_angles[i];
            } else {
                optimize(state, current_angles[i]);
            }

            states.at(i) = state;
        }

        return states;
    }

    void SwerveKinematics::optimize(ModuleState& state, const double current_angle) {
        if (const double angle_diff = angles::shortest_angular_distance(current_angle, state.angle); std::abs(angle_diff) > std::numbers::pi / 2) {
            state.speed = -state.speed;
            state.angle = angles::normalize_angle(state.angle + std::numbers::pi);
        }
    }
}
