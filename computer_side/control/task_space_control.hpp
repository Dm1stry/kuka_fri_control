#ifndef TASK_SPACE_CONTROL_HPP
#define TASK_SPACE_CONTROL_HPP

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/context.h>

#include <memory>
#include <string>
#include <iostream>

#include "control_interface.hpp"

namespace control
{
    class TaskSpaceControl : public IControl
    {
    public:
        TaskSpaceControl(const std::string &urdf_name,
                         const std::string &base_frame = "iiwa_link_0",
                         const std::string &end_effector_frame = "iiwa_link_ee",
                         double time_tick = 0.005);

        void setStiffness(const Eigen::Matrix<double,6,1> &stiffness);
        void setDamping(const Eigen::Matrix<double,6,1> &damping);
        void setTargetPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation);
        void setTargetWrench(const Eigen::Matrix<double,6,1> &wrench);
        void setNullspaceTarget(const Eigen::Array<double,N_JOINTS,1> &q_ref);
        void setNullspaceGains(const Eigen::Array<double,N_JOINTS,1> &stiffness,
                               const Eigen::Array<double,N_JOINTS,1> &damping);
        void useBiasCompensation(bool enabled);

        int updateTarget(const Eigen::Vector3d &target_pos, const Eigen::Matrix<double,3,3> &target_rot) override;
        void updateCurrentState(const Eigen::Array<double,N_JOINTS,1> &current_thetta,
                                const Eigen::Array<double,N_JOINTS,1> &current_torque) override;
        Eigen::Array<double,N_JOINTS,1> getTorque() override;
        Eigen::Array<double,N_JOINTS,1> getNextPoint() override;
        Eigen::Array<double,N_JOINTS,1> getTorque(const Eigen::Array<double,N_JOINTS,1> &q,
                                                  const Eigen::Array<double,N_JOINTS,1> &dq);
        Eigen::Array<double,N_JOINTS,1> getTargetThetta() const override;

        Eigen::Vector3d getCurrentPosition() const override;
        Eigen::Matrix3d getCurrentRotation() const override;
        Eigen::Matrix<double,6,1> getForce() const override;
        Eigen::Matrix<double,6,1> getTaskError() const;
        int getState() const override;

    private:
        Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix, double tolerance = 1e-6) const;
        Eigen::Matrix<double,6,N_JOINTS> calcJacobian(const Eigen::Array<double,N_JOINTS,1> &q);
        Eigen::Matrix<double,6,1> calcTaskError(const Eigen::Matrix3d &current_rotation,
                                                const Eigen::Vector3d &current_position) const;
        Eigen::Array<double,N_JOINTS,1> calcBiasTorque(const Eigen::Array<double,N_JOINTS,1> &q,
                                                       const Eigen::Array<double,N_JOINTS,1> &dq);

        drake::multibody::MultibodyPlant<double> plant_;
        std::unique_ptr<drake::systems::Context<double>> context_;

        std::string base_frame_;
        std::string end_effector_frame_;
        double time_tick_ = 0.005;

        Eigen::Vector3d target_position_;
        Eigen::Matrix3d target_rotation_;
        Eigen::Vector3d current_position_;
        Eigen::Matrix3d current_rotation_;

        Eigen::Matrix<double,6,1> stiffness_;
        Eigen::Matrix<double,6,1> damping_;
        Eigen::Matrix<double,6,1> target_wrench_;
        Eigen::Matrix<double,6,1> task_error_;
        Eigen::Matrix<double,6,1> force_;

        Eigen::Array<double,N_JOINTS,1> current_q_;
        Eigen::Array<double,N_JOINTS,1> current_dq_;
        Eigen::Array<double,N_JOINTS,1> previous_q_;
        Eigen::Array<double,N_JOINTS,1> target_torque_;
        Eigen::Array<double,N_JOINTS,1> q_ref_;
        Eigen::Array<double,N_JOINTS,1> nullspace_stiffness_;
        Eigen::Array<double,N_JOINTS,1> nullspace_damping_;

        bool use_bias_compensation_ = true;
        bool state_initialized_ = false;
        int state_ = 1;
    };
}

#endif
