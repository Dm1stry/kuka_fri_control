#include "trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace trajectory
{
    TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d &initial_position,
                                             const Eigen::Matrix3d &initial_rotation,
                                             TrajectoryType type,
                                             double amplitude,
                                             double frequency,
                                             double warmup_time,
                                             double rotation_amplitude):
    initial_position_(initial_position),
    initial_rotation_(initial_rotation),
    type_(type),
    amplitude_(amplitude),
    frequency_(frequency),
    warmup_time_(warmup_time),
    rotation_amplitude_(rotation_amplitude),
    start_time_(std::chrono::steady_clock::now())
    {
    }

    void TrajectoryGenerator::reset(const Eigen::Vector3d &initial_position,
                                    const Eigen::Matrix3d &initial_rotation)
    {
        initial_position_ = initial_position;
        initial_rotation_ = initial_rotation;
        start_time_ = std::chrono::steady_clock::now();
    }

    bool TrajectoryGenerator::getTarget(Eigen::Vector3d &target_position,
                                        Eigen::Matrix3d &target_rotation)
    {
        const auto now = std::chrono::steady_clock::now();
        const double time = std::chrono::duration<double>(now - start_time_).count();
        const double phase = 2.0 * std::numbers::pi * frequency_ * time;
        const double ramp = warmup_time_ > 0.0
            ? smoothStep(std::clamp(time / warmup_time_, 0.0, 1.0))
            : 1.0;

        target_position = initial_position_;
        target_rotation = initial_rotation_;

        switch (type_)
        {
            case TrajectoryType::CircleXY:
                target_position.x() += ramp * amplitude_ * (1.0 - std::cos(phase));
                target_position.y() += ramp * amplitude_ * std::sin(phase);
                break;

            case TrajectoryType::CircleXYZ:
                target_position.x() += ramp * amplitude_ * (1.0 - std::cos(phase));
                target_position.y() += ramp * amplitude_ * std::sin(phase);
                target_position.z() += ramp * amplitude_ * std::sin(phase + std::numbers::pi / 2.0);
                break;

            case TrajectoryType::SquareXYZ:
            {
                const double cycle = std::fmod(frequency_ * time, 1.0);
                const double square_phase = 4.0 * cycle;
                const int side = static_cast<int>(square_phase);
                const double side_progress = square_phase - side;
                const double side_length = 2.0 * amplitude_;
                double x_offset = 0.0;
                double y_offset = 0.0;

                switch (side)
                {
                    case 0:
                        x_offset = side_length * side_progress;
                        y_offset = 0.0;
                        break;

                    case 1:
                        x_offset = side_length;
                        y_offset = side_length * side_progress;
                        break;

                    case 2:
                        x_offset = side_length * (1.0 - side_progress);
                        y_offset = side_length;
                        break;

                    default:
                        x_offset = 0.0;
                        y_offset = side_length * (1.0 - side_progress);
                        break;
                }

                target_position.x() += ramp * x_offset;
                target_position.y() += ramp * y_offset;
                target_position.z() += ramp * amplitude_ * std::sin(phase);
                break;
            }

            case TrajectoryType::LineY:
                target_position.y() += ramp * amplitude_ * std::sin(phase);
                break;

            case TrajectoryType::SineZ:
                target_position.z() += ramp * amplitude_ * std::sin(phase);
                break;

            case TrajectoryType::SineRotationZ:
            {
                const double angle = ramp * amplitude_ * std::sin(phase);
                const Eigen::Matrix3d rotation_delta =
                    Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                target_rotation = initial_rotation_ * rotation_delta;
                break;
            }

            case TrajectoryType::CircleXYRotationXY:
            {
                target_position.x() += ramp * amplitude_ * (1.0 - std::cos(phase));
                target_position.y() += ramp * amplitude_ * std::sin(phase);

                const double angle_x = ramp * rotation_amplitude_ * std::sin(phase);
                const double angle_y = ramp * rotation_amplitude_ * std::cos(phase);
                const Eigen::Matrix3d rotation_delta =
                    Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                    Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY()).toRotationMatrix();
                target_rotation = initial_rotation_ * rotation_delta;
                break;
            }
        }

        return true;
    }

    double TrajectoryGenerator::smoothStep(double value)
    {
        value = std::clamp(value, 0.0, 1.0);
        return value * value * (3.0 - 2.0 * value);
    }
}
