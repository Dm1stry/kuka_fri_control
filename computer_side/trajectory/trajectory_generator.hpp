#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace trajectory
{
    enum class TrajectoryType
    {
        CircleXY,
        LineY,
        SineZ,
        SineRotationZ,
        CircleXYRotationXY,
        CircleXYZ
    };

    class TrajectoryGenerator
    {
    public:
        TrajectoryGenerator(const Eigen::Vector3d &initial_position,
                            const Eigen::Matrix3d &initial_rotation,
                            TrajectoryType type = TrajectoryType::CircleXY,
                            double amplitude = 0.03,
                            double frequency = 0.05,
                            double warmup_time = 2.0,
                            double rotation_amplitude = 0.17453292519943295);

        void reset(const Eigen::Vector3d &initial_position,
                   const Eigen::Matrix3d &initial_rotation);

        bool getTarget(Eigen::Vector3d &target_position,
                       Eigen::Matrix3d &target_rotation);

    private:
        static double smoothStep(double value);

        Eigen::Vector3d initial_position_;
        Eigen::Matrix3d initial_rotation_;
        TrajectoryType type_;
        double amplitude_;
        double frequency_;
        double warmup_time_;
        double rotation_amplitude_;
        std::chrono::steady_clock::time_point start_time_;
    };
}

#endif
