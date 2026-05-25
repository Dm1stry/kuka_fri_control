#include "task_space_control.hpp"

#include <drake/multibody/tree/multibody_forces.h>

#include <algorithm>
#include <cmath>

using namespace control;

TaskSpaceControl::TaskSpaceControl(const std::string &urdf_name,
                                   const std::string &base_frame,
                                   const std::string &end_effector_frame,
                                   double time_tick):
plant_(0.0),
base_frame_(base_frame),
end_effector_frame_(end_effector_frame),
time_tick_(time_tick)
{
    drake::multibody::Parser parser(&plant_);
    parser.AddModels(urdf_name);
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    target_position_.setZero();
    target_rotation_.setIdentity();
    virtual_target_position_.setZero();
    virtual_target_rotation_.setIdentity();
    current_position_.setZero();
    current_rotation_.setIdentity();
    task_error_.setZero();
    force_.setZero();
    target_wrench_.setZero();

    stiffness_ << 80., 80., 80., 500., 500., 500.;
    damping_ << 8., 8., 8., 45., 45., 45.;

    current_q_.setZero();
    current_dq_.setZero();
    previous_q_.setZero();
    virtual_q_.setZero();
    target_torque_.setZero();
    q_ref_.setZero();
    nullspace_stiffness_.setZero();
    nullspace_damping_.setZero();
}

void TaskSpaceControl::setStiffness(const Eigen::Matrix<double,6,1> &stiffness)
{
    stiffness_ = stiffness;
}

void TaskSpaceControl::setDamping(const Eigen::Matrix<double,6,1> &damping)
{
    damping_ = damping;
}

void TaskSpaceControl::setTargetPose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation)
{
    target_position_ = position;
    target_rotation_ = rotation;
}

void TaskSpaceControl::setTargetWrench(const Eigen::Matrix<double,6,1> &wrench)
{
    target_wrench_ = wrench;
}

void TaskSpaceControl::setNullspaceTarget(const Eigen::Array<double,N_JOINTS,1> &q_ref)
{
    q_ref_ = q_ref;
}

void TaskSpaceControl::setNullspaceGains(const Eigen::Array<double,N_JOINTS,1> &stiffness,
                                         const Eigen::Array<double,N_JOINTS,1> &damping)
{
    nullspace_stiffness_ = stiffness;
    nullspace_damping_ = damping;
}

void TaskSpaceControl::useBiasCompensation(bool enabled)
{
    use_bias_compensation_ = enabled;
}

int TaskSpaceControl::updateTarget(const Eigen::Vector3d &target_pos, const Eigen::Matrix<double,3,3> &target_rot)
{
    setTargetPose(target_pos, target_rot);
    state_ = 1;
    return state_;
}

void TaskSpaceControl::updateCurrentState(const Eigen::Array<double,N_JOINTS,1> &current_thetta,
                                          const Eigen::Array<double,N_JOINTS,1> &current_torque)
{
    if (!state_initialized_)
    {
        current_dq_.setZero();
        state_initialized_ = true;
    }
    else
    {
        current_dq_ = (current_thetta - previous_q_) / time_tick_;
    }

    current_q_ = current_thetta;
    previous_q_ = current_thetta;

    plant_.SetPositions(context_.get(), current_q_.matrix());
    plant_.SetVelocities(context_.get(), current_dq_.matrix());

    const auto &base_frame = plant_.GetFrameByName(base_frame_);
    const auto &end_effector_frame = plant_.GetFrameByName(end_effector_frame_);
    const auto X_BE = plant_.CalcRelativeTransform(*context_, base_frame, end_effector_frame);

    current_position_ = X_BE.translation();
    current_rotation_ = X_BE.rotation().matrix();

    if (!virtual_target_initialized_)
    {
        virtual_q_ = current_q_;
        virtual_target_position_ = current_position_;
        virtual_target_rotation_ = current_rotation_;
        q_ref_ = virtual_q_;
        virtual_target_initialized_ = true;
    }

    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(current_q_);
    force_ = J.transpose().fullPivHouseholderQr().solve(current_torque.matrix());
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getTorque()
{
    updateVirtualTarget();
    target_torque_ = getTorque(current_q_, current_dq_);
    return target_torque_;
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getNextPoint()
{
    updateVirtualTarget();
    return virtual_q_;
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getTorque(
    const Eigen::Array<double,N_JOINTS,1> &q,
    const Eigen::Array<double,N_JOINTS,1> &dq)
{
    plant_.SetPositions(context_.get(), q.matrix());
    plant_.SetVelocities(context_.get(), dq.matrix());

    const auto &base_frame = plant_.GetFrameByName(base_frame_);
    const auto &end_effector_frame = plant_.GetFrameByName(end_effector_frame_);
    const auto X_BE = plant_.CalcRelativeTransform(*context_, base_frame, end_effector_frame);

    current_position_ = X_BE.translation();
    current_rotation_ = X_BE.rotation().matrix();
    task_error_ = calcTaskError(current_rotation_, current_position_);

    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(q);
    const Eigen::Matrix<double,6,1> task_velocity = J * dq.matrix();
    const Eigen::Matrix<double,6,1> task_wrench =
        stiffness_.cwiseProduct(task_error_) - damping_.cwiseProduct(task_velocity) + target_wrench_;

    Eigen::Matrix<double,N_JOINTS,1> tau = J.transpose() * task_wrench;

    // if ((nullspace_stiffness_.abs().sum() > 0.) || (nullspace_damping_.abs().sum() > 0.))
    // {
    //     const Eigen::MatrixXd J_pinv = pseudoInverse(J);
    //     const Eigen::Matrix<double,N_JOINTS,N_JOINTS> nullspace =
    //         Eigen::Matrix<double,N_JOINTS,N_JOINTS>::Identity() - J_pinv * J;
    //     const Eigen::Matrix<double,N_JOINTS,1> tau_secondary =
    //         (nullspace_stiffness_ * (q_ref_ - q) - nullspace_damping_ * dq).matrix();

    //     tau += nullspace * tau_secondary;
    // }

    // if (use_bias_compensation_)
    // {
    //     tau += calcBiasTorque(q, dq).matrix();
    // }

    return tau.array();
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getTargetThetta() const
{
    return virtual_q_;
}

Eigen::Vector3d TaskSpaceControl::getCurrentPosition() const
{
    return current_position_;
}

Eigen::Matrix3d TaskSpaceControl::getCurrentRotation() const
{
    return current_rotation_;
}

Eigen::Matrix<double,6,1> TaskSpaceControl::getTaskError() const
{
    return task_error_;
}

Eigen::Matrix<double,6,1> TaskSpaceControl::getForce() const
{
    return force_;
}

int TaskSpaceControl::getState() const
{
    return state_;
}

Eigen::MatrixXd TaskSpaceControl::pseudoInverse(const Eigen::MatrixXd &matrix, double tolerance) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto &singular_values = svd.singularValues();
    Eigen::MatrixXd singular_values_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (int i = 0; i < singular_values.size(); ++i)
    {
        if (singular_values[i] > tolerance)
        {
            singular_values_inv(i, i) = 1. / singular_values[i];
        }
    }

    return svd.matrixV() * singular_values_inv * svd.matrixU().transpose();
}

Eigen::Matrix<double,6,N_JOINTS> TaskSpaceControl::calcJacobian(
    const Eigen::Array<double,N_JOINTS,1> &q)
{
    plant_.SetPositions(context_.get(), q.matrix());

    const auto &base_frame = plant_.GetFrameByName(base_frame_);
    const auto &end_effector_frame = plant_.GetFrameByName(end_effector_frame_);
    Eigen::Matrix<double,6,N_JOINTS> J;

    plant_.CalcJacobianSpatialVelocity(
        *context_,
        drake::multibody::JacobianWrtVariable::kV,
        end_effector_frame,
        Eigen::Vector3d::Zero(),
        base_frame,
        base_frame,
        &J);

    return J;
}

Eigen::Matrix<double,6,1> TaskSpaceControl::calcTaskError(
    const Eigen::Matrix3d &current_rotation,
    const Eigen::Vector3d &current_position) const
{
    return calcTaskErrorToTarget(
        current_rotation,
        current_position,
        virtual_target_rotation_,
        virtual_target_position_);
}

Eigen::Matrix<double,6,1> TaskSpaceControl::calcTaskErrorToTarget(
    const Eigen::Matrix3d &current_rotation,
    const Eigen::Vector3d &current_position,
    const Eigen::Matrix3d &target_rotation,
    const Eigen::Vector3d &target_position) const
{
    Eigen::Matrix<double,6,1> error;
    Eigen::AngleAxisd angle_axis(target_rotation * current_rotation.transpose());
    error.head<3>() = angle_axis.angle() * angle_axis.axis();
    error.tail<3>() = target_position - current_position;

    return error;
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getJointDelta(
    const Eigen::Array<double,N_JOINTS,1> &target_q,
    const Eigen::Array<double,N_JOINTS,1> &current_q) const
{
    Eigen::Array<double,N_JOINTS,1> delta = target_q - current_q;
    Eigen::Array<double,N_JOINTS,1> step;

    for (int i = 0; i < N_JOINTS; ++i)
    {
        const double abs_delta = std::abs(delta[i]);

        if (abs_delta <= e_min_)
        {
            step[i] = 0.;
        }
        else if (abs_delta < e_max_)
        {
            const double ratio = (abs_delta - e_min_) / (e_max_ - e_min_);
            const double velocity = v_min_ + (v_max_ - v_min_) * ratio;
            step[i] = std::min(velocity, abs_delta) * (delta[i] > 0. ? 1. : -1.);
        }
        else
        {
            step[i] = std::min(v_max_, abs_delta) * (delta[i] > 0. ? 1. : -1.);
        }
    }

    return step;
}

void TaskSpaceControl::updateVirtualTarget()
{
    if (!virtual_target_initialized_)
    {
        return;
    }

    const Eigen::Matrix<double,6,1> error_to_goal = calcTaskErrorToTarget(
        virtual_target_rotation_,
        virtual_target_position_,
        target_rotation_,
        target_position_);

    Eigen::Matrix<double,6,1> step_error;
    step_error.setZero();

    const double angular_error = error_to_goal.head<3>().norm();
    if (angular_error > target_rot_eps_)
    {
        const double angular_step = std::min(
            angular_step_max_,
            std::max(angular_step_min_, angular_error));
        step_error.head<3>() = error_to_goal.head<3>() / angular_error *
                               std::min(angular_step, angular_error);
    }

    const double linear_error = error_to_goal.tail<3>().norm();
    if (linear_error > target_pos_eps_)
    {
        const double linear_step = std::min(
            linear_step_max_,
            std::max(linear_step_min_, linear_error));
        step_error.tail<3>() = error_to_goal.tail<3>() / linear_error *
                               std::min(linear_step, linear_error);
    }

    if (step_error.isZero(0.))
    {
        return;
    }

    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(virtual_q_);
    const Eigen::Matrix<double,N_JOINTS,1> q_delta = pseudoInverse(J) * step_error;
    const Eigen::Array<double,N_JOINTS,1> q_command = virtual_q_ + q_delta.array();

    virtual_q_ += getJointDelta(q_command, virtual_q_);
    q_ref_ = virtual_q_;

    plant_.SetPositions(context_.get(), virtual_q_.matrix());
    const auto &base_frame = plant_.GetFrameByName(base_frame_);
    const auto &end_effector_frame = plant_.GetFrameByName(end_effector_frame_);
    const auto X_BE = plant_.CalcRelativeTransform(*context_, base_frame, end_effector_frame);

    virtual_target_position_ = X_BE.translation();
    virtual_target_rotation_ = X_BE.rotation().matrix();
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::calcBiasTorque(
    const Eigen::Array<double,N_JOINTS,1> &q,
    const Eigen::Array<double,N_JOINTS,1> &dq)
{
    plant_.SetPositions(context_.get(), q.matrix());
    plant_.SetVelocities(context_.get(), dq.matrix());

    drake::multibody::MultibodyForces<double> external_forces(plant_);
    const Eigen::VectorXd zero_acceleration = Eigen::VectorXd::Zero(plant_.num_velocities());
    const Eigen::VectorXd tau_bias = plant_.CalcInverseDynamics(*context_, zero_acceleration, external_forces);

    return tau_bias.array();
}
