#ifndef CONTROL_INTERFACE_HPP
#define CONTROL_INTERFACE_HPP

#include <Eigen/Dense>

namespace control
{
    constexpr int N_JOINTS = 7;

    using JointArray = Eigen::Array<double,N_JOINTS,1>;
    using CartesianVector = Eigen::Matrix<double,6,1>;
    using RotationMatrix = Eigen::Matrix<double,3,3>;

    class IControl
    {
    public:
        virtual ~IControl() = default;

        virtual int updateTarget(const Eigen::Vector3d &target_pos, const RotationMatrix &target_rot) = 0;
        virtual void updateCurrentState(const JointArray &current_thetta, const JointArray &current_torque) = 0;

        virtual JointArray getTorque() = 0;
        virtual JointArray getTargetThetta() const = 0;
        virtual Eigen::Vector3d getCurrentPosition() const = 0;
        virtual RotationMatrix getCurrentRotation() const = 0;
        virtual CartesianVector getForce() const = 0;
        virtual int getState() const = 0;
    };
}

#endif
