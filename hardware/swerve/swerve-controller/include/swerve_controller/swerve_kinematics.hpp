#pragma once

#include <array>
#include <Eigen/Core>

namespace swerve_controller {
    struct ModulePosition {
        double x;
        double y;
    };

    struct ModuleState {
        double speed;
        double angle;
    };

    class SwerveKinematics {
    public:
        explicit SwerveKinematics(const Eigen::Matrix<double, 4, 2>& module_positions);

        [[nodiscard]] std::array<ModuleState, 4> inverse(
            const Eigen::Vector2d& linear_velocity,
            double angular_velocity,
            const Eigen::Vector4d& current_angles
        ) const;

    private:
        Eigen::Matrix<double, 4, 2> module_positions;
        Eigen::Matrix<double, 2, 4> module_tangents;

        static void optimize(ModuleState& state, double current_angle);
    };
}
