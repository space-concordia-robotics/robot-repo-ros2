#pragma once

/**
 * @file swerve_kinematics.hpp
 * @brief Inverse kinematics solver and state structures for a 4-wheel independent steering (4WIS) swerve drive.
 *
 * Computes individual wheel speed and steering angle targets from desired chassis
 * linear (vx, vy) and angular (omega_z) velocities, including wheel angle optimization
 * to minimize steering rotation.
 */

#include <array>
#include <Eigen/Core>

namespace swerve_controller {
    /**
     * @brief 2D coordinates (in meters) of a swerve module relative to the robot's base_link origin.
     * +x points forward, +y points left (standard ROS REP-103 coordinate frame).
     */
    struct ModulePosition {
        double x; ///< Longitudinal offset from base_link center (m)
        double y; ///< Lateral offset from base_link center (m)
    };

    /**
     * @brief Desired or measured kinematic state of a single swerve module.
     */
    struct ModuleState {
        double speed; ///< Linear tangential wheel ground speed (m/s)
        double angle; ///< Steering yaw angle relative to +x axis (radians)
    };

    /**
     * @brief Kinematics solver for a 4-module swerve drivetrain.
     */
    class SwerveKinematics {
    public:
        /**
         * @brief Constructs the kinematics solver and precomputes tangential rotation vectors.
         * @param module_positions 4x2 matrix where each row is the (x, y) position of a module
         *                         (ordered: front_left, front_right, rear_left, rear_right).
         */
        explicit SwerveKinematics(const Eigen::Matrix<double, 4, 2>& module_positions);

        /**
         * @brief Computes inverse kinematics: chassis twist -> 4 module states (speed & steering angle).
         *
         * @param linear_velocity  Desired chassis linear velocity vector [vx, vy]^T (m/s) in base_link frame.
         * @param angular_velocity Desired chassis yaw rate omega_z (rad/s) around base_link Z axis.
         * @param current_angles   Current measured steering angles [fl, fr, rl, rr]^T (rad) used for
         *                         holding angle at zero speed and optimizing steering rotation.
         * @return std::array<ModuleState, 4> Target speed (m/s) and steering angle (rad) for each module.
         */
        [[nodiscard]] std::array<ModuleState, 4> inverse(
            const Eigen::Vector2d& linear_velocity,
            double angular_velocity,
            const Eigen::Vector4d& current_angles
        ) const;

    private:
        /// 4x2 matrix storing the (x, y) position of each swerve module relative to base_link.
        Eigen::Matrix<double, 4, 2> module_positions;

        /// Precomputed 2x4 matrix where each column i is [-y_i, x_i]^T, representing the
        /// tangential velocity direction per unit of chassis angular velocity (omega_z).
        Eigen::Matrix<double, 2, 4> module_tangents;

        /**
         * @brief Optimizes a module's target state so the steering joint never rotates more than 90 degrees (pi/2 rad).
         *
         * If the shortest angular distance from `current_angle` to `state.angle` exceeds pi/2,
         * the target steering angle is flipped by pi radians and the wheel drive speed is negated.
         *
         * @param state         In/out target module state (speed and angle) to optimize.
         * @param current_angle Current measured steering angle of the module (radians).
         */
        static void optimize(ModuleState& state, double current_angle);
    };
}
