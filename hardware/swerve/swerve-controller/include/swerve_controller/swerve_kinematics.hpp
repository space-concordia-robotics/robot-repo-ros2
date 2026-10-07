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
        static constexpr auto module_count = 4;

        using ModulePositions = Eigen::Matrix<double, module_count, 2>;
        using ModuleAngles = Eigen::Vector<double, module_count>;
        using ModuleStates = std::array<ModuleState, module_count>;

        explicit SwerveKinematics(const ModulePositions& module_positions);

        [[nodiscard]] ModuleStates inverse(
            const Eigen::Vector2d& linear_velocity,
            double angular_velocity,
            const ModuleAngles& current_angles
        ) const;

    private:
        ModulePositions module_positions;
        Eigen::Matrix<double, 2, module_count> module_tangents;

        static void optimize(ModuleState& state, double current_angle);
    };
}
