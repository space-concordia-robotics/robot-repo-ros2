/**
 * @file swerve_kinematics.cpp
 * @brief Implementation of the 4-wheel swerve drive inverse kinematics and steering angle optimization.
 */

#include "swerve_controller/swerve_kinematics.hpp"

#include <cmath>
#include <ranges>
#include <angles/angles.h>

namespace swerve_controller {
    SwerveKinematics::SwerveKinematics(const Eigen::Matrix<double, 4, 2>& module_positions)
        : module_positions(module_positions) {
        // 2D 90-degree counter-clockwise rotation matrix operator (in right-multiply form)
        // Used to map each module position (x_i, y_i) to its rotational tangent vector (-y_i, x_i).
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

        // Compute the rotational velocity contribution v_rot_i = omega_z x r_i = [-omega_z * y_i, omega_z * x_i]^T
        // for all 4 modules simultaneously via matrix-scalar multiplication.
        const auto wheel_velocities = module_tangents * angular_velocity;

        for (auto&& [i, state] : std::views::enumerate(states)) {
            // Total 2D velocity vector at module i is the sum of linear chassis velocity and rotational velocity
            const auto wheel_velocity = wheel_velocities.col(i) + linear_velocity;

            // Convert Cartesian velocity vector (vx_i, vy_i) to polar form (speed, steering angle)
            state.speed = wheel_velocity.norm();
            state.angle = std::atan2(wheel_velocity.y(), wheel_velocity.x());

            // Deadband check: if commanded wheel speed is effectively zero, keep the current steering angle
            // so the wheels don't snap back to 0 radians when the robot stops.
            if (state.speed <= 1e-9) {
                state.speed = 0.0;
                state.angle = current_angles[i];
            } else {
                // Minimize steering rotation by reversing wheel velocity if target angle is > 90 deg away
                optimize(state, current_angles[i]);
            }
        }

        return states;
    }

    void SwerveKinematics::optimize(ModuleState& state, const double current_angle) {
        // TODO 2026-10-14 (Will Free): there is something wrong here causing it to sometimes flip when it's not supposed to. idk what.

        // If the shortest angular path to the target angle exceeds 90 degrees (pi/2 rad),
        // it is faster to rotate to the opposite angle (+pi) and spin the drive wheel in reverse.
        if (const double angle_diff = angles::shortest_angular_distance(current_angle, state.angle); std::abs(angle_diff) > std::numbers::pi / 2) {
            state.speed = -state.speed;
            state.angle = angles::normalize_angle(state.angle + std::numbers::pi);
        }
    }
}
