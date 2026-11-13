#include "swerve_controller/swerve_kinematics.hpp"

#include <cmath>
#include <ranges>
#include <angles/angles.h>

namespace swerve_controller {
    SwerveKinematics::SwerveKinematics(const Eigen::Matrix<double, 4, 2>& module_positions)
        : module_positions(module_positions) {
        static const auto rotation_matrix = Eigen::Matrix2d({
            {0.0, 1.0},
            {-1.0, 0.0},
        });

        /*
        flips the first & second columns, multiplying the second one by -1, then transpose it.

        e.g. given a wheel base of 4 and a track width of 3, you would get
        [  2   1.5 ]             [ -1.5   2 ]
        [  2  -1.5 ]           = [  1.5   2 ]
        [ -2   1.5 ] [  0  1 ]   [ -1.5  -2 ]
        [ -2  -1.5 ] [ -1  0 ]   [  1.5  -2 ]

        [ -1.5   2 ]^T
        [  1.5   2 ]
        [ -1.5  -2 ]   = [ -1.5  1.5  -1.5  1.5 ]
        [  1.5  -2 ]     [  2    2    -2    -2  ]

        this is precomputed to improve the performance of the IK
        */

        const auto transformed = module_positions * rotation_matrix;
        module_tangents = transformed.transpose();
    }

    std::array<ModuleState, 4> SwerveKinematics::inverse(
        const Eigen::Vector2d& linear_velocity,
        const double angular_velocity,
        const Eigen::Vector4d& current_angles
    ) const {
        std::array<ModuleState, 4> states = {};

        const auto wheel_velocities = module_tangents * angular_velocity;

        for (auto&& [i, state] : std::views::enumerate(states)) {
            const auto wheel_velocity = wheel_velocities.col(i) + linear_velocity;

            state.speed = wheel_velocity.norm();
            state.angle = std::atan2(wheel_velocity.y(), wheel_velocity.x());

            if (state.speed <= 1e-9) {
                state.speed = 0.0;
                state.angle = current_angles[i];
            } else {
                optimize(state, current_angles[i]);
            }
        }

        return states;
    }

    void SwerveKinematics::optimize(ModuleState& state, const double current_angle) {
        // TODO 2026-10-14 (Will Free): there is something wrong here causing it to sometimes flip when it's not supposed to. idk what.

        if (const double angle_diff = angles::shortest_angular_distance(current_angle, state.angle); std::abs(angle_diff) > std::numbers::pi / 2) {
            state.speed = -state.speed;
            state.angle = angles::normalize_angle(state.angle + std::numbers::pi);
        }
    }
}
