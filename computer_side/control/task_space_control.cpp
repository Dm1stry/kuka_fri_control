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
    joint_min_.setZero();
    joint_max_.setZero();
    joint_limit_stiffness_.setZero();
    joint_limit_damping_.setZero();
    joint_limit_torque_max_.setZero();
    cabinet_stiffness_.setZero();
    delta_min_.setZero();
    delta_max_.setZero();

    nullspace_stiffness_ << 60., 60., 60., 45., 35., 35., 35.;
    nullspace_damping_ << 8.0, 8.0, 8.0, 6.0, 5.0, 5.0, 5.0;
    cabinet_stiffness_ << 500., 500., 500., 300., 300., 150., 150.;
    delta_max_ = max_joint_velocity_ * time_tick_; // * cabinet_stiffness_ / cabinet_stiffness_.maxCoeff();
    delta_min_ = delta_max_ * min_delta_ratio_;
    joint_min_ << -168. * M_PI / 180., -118. * M_PI / 180., -168. * M_PI / 180.,
                  -118. * M_PI / 180., -168. * M_PI / 180., -118. * M_PI / 180.,
                  -173. * M_PI / 180.;
    joint_max_ << 168. * M_PI / 180., 118. * M_PI / 180., 168. * M_PI / 180.,
                  118. * M_PI / 180., 168. * M_PI / 180., 118. * M_PI / 180.,
                  173. * M_PI / 180.;
    joint_limit_stiffness_ << 180., 180., 180., 140., 120., 120., 120.;
    joint_limit_damping_ << 18., 18., 18., 14., 12., 12., 12.;
    joint_limit_torque_max_ << 80., 80., 80., 65., 55., 45., 35.;
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
    if (!target_filter_initialized_)
    {
        target_position_ = position;
        target_rotation_ = rotation;
        target_filter_initialized_ = true;
        return;
    }

    const double new_target_weight = 1. - target_filter_alpha_;
    target_position_ = target_filter_alpha_ * target_position_ + new_target_weight * position;

    Eigen::Quaterniond current_target(target_rotation_);
    Eigen::Quaterniond new_target(rotation);
    current_target.normalize();
    new_target.normalize();

    if (current_target.dot(new_target) < 0.)
    {
        new_target.coeffs() *= -1.;
    }

    target_rotation_ = current_target.slerp(new_target_weight, new_target).normalized().toRotationMatrix();
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

    if ((nullspace_stiffness_.abs().sum() > 0.) ||
        (nullspace_damping_.abs().sum() > 0.) ||
        use_singularity_avoidance_)
    {
        const Eigen::Matrix<double,N_JOINTS,6> J_pinv = dampedPseudoInverse(J);
        const Eigen::Matrix<double,N_JOINTS,N_JOINTS> nullspace =
            Eigen::Matrix<double,N_JOINTS,N_JOINTS>::Identity() - J_pinv * J;
        const Eigen::Matrix<double,N_JOINTS,1> tau_secondary =
            (nullspace_stiffness_ * (q_ref_ - q) - nullspace_damping_ * dq +
             calcJointLimitTorque(q, dq)); // + calcSingularityAvoidanceTorque(q, dq)).matrix();

        tau += nullspace * tau_secondary;
    }

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

Eigen::Matrix<double,N_JOINTS,6> TaskSpaceControl::dampedPseudoInverse(
    const Eigen::Matrix<double,6,N_JOINTS> &matrix) const
{
    const Eigen::Matrix<double,6,6> damping =
        dls_lambda_ * dls_lambda_ * Eigen::Matrix<double,6,6>::Identity();

    return matrix.transpose() * (matrix * matrix.transpose() + damping).inverse();
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
    if (delta.abs().maxCoeff() <= e_min_)
    {
        return Eigen::Array<double,N_JOINTS,1>::Zero();
    }

    double max_ratio = 1.;

    for (int i = 0; i < N_JOINTS; ++i)
    {
        if (delta_max_[i] > 0.)
        {
            max_ratio = std::max(max_ratio, std::abs(delta[i]) / delta_max_[i]);
        }
    }

    return delta / max_ratio;
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
        const double angular_step = std::min(angular_step_max_, std::max(angular_step_min_, angular_error));

        step_error.head<3>() = error_to_goal.head<3>() / angular_error * std::min(angular_step, angular_error);
    }

    const double linear_error = error_to_goal.tail<3>().norm();
    if (linear_error > target_pos_eps_)
    {
        const double linear_step = std::min(linear_step_max_,std::max(linear_step_min_, linear_error));

        step_error.tail<3>() = error_to_goal.tail<3>() / linear_error * std::min(linear_step, linear_error);
    }

    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(virtual_q_);
    const Eigen::Matrix<double,N_JOINTS,6> J_pinv = dampedPseudoInverse(J);
    const Eigen::Matrix<double,N_JOINTS,N_JOINTS> nullspace =
        Eigen::Matrix<double,N_JOINTS,N_JOINTS>::Identity() - J_pinv * J;
    const Eigen::Array<double,N_JOINTS,1> q_secondary =
        getJointDelta(q_ref_, virtual_q_) +
        calcJointLimitDelta(virtual_q_);
        // calcSingularityAvoidanceDelta(virtual_q_);
    const Eigen::Matrix<double,N_JOINTS,1> q_delta =
        J_pinv * step_error + nullspace * q_secondary.matrix();
    const Eigen::Array<double,N_JOINTS,1> q_command = virtual_q_ + q_delta.array();

    virtual_q_ += getJointDelta(q_command, virtual_q_);

    virtual_target_position_ += step_error.tail<3>();

    const double angular_step_norm = step_error.head<3>().norm();
    if (angular_step_norm > 0.)
    {
        const Eigen::AngleAxisd rotation_step(
            angular_step_norm,
            step_error.head<3>() / angular_step_norm);
        virtual_target_rotation_ = rotation_step.toRotationMatrix() * virtual_target_rotation_;
    }

    plant_.SetPositions(context_.get(), virtual_q_.matrix());
    const auto &base_frame = plant_.GetFrameByName(base_frame_);
    const auto &end_effector_frame = plant_.GetFrameByName(end_effector_frame_);
    const auto X_BE = plant_.CalcRelativeTransform(*context_, base_frame, end_effector_frame);
    const Eigen::Vector3d actual_virtual_position = X_BE.translation();
    const Eigen::Matrix3d actual_virtual_rotation = X_BE.rotation().matrix();

    virtual_target_position_ =
        (1. - virtual_sync_alpha_) * virtual_target_position_ +
        virtual_sync_alpha_ * actual_virtual_position;

    Eigen::Quaterniond desired_virtual_rotation(virtual_target_rotation_);
    Eigen::Quaterniond actual_virtual_rotation_q(actual_virtual_rotation);
    desired_virtual_rotation.normalize();
    actual_virtual_rotation_q.normalize();

    if (desired_virtual_rotation.dot(actual_virtual_rotation_q) < 0.)
    {
        actual_virtual_rotation_q.coeffs() *= -1.;
    }

    virtual_target_rotation_ =
        desired_virtual_rotation
            .slerp(virtual_sync_alpha_, actual_virtual_rotation_q)
            .normalized()
            .toRotationMatrix();
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::calcJointLimitDelta(
    const Eigen::Array<double,N_JOINTS,1> &q) const
{
    Eigen::Array<double,N_JOINTS,1> delta;
    delta.setZero();

    for (int i = 0; i < N_JOINTS; ++i)
    {
        const double lower_soft_limit = joint_min_[i] + joint_limit_margin_;
        const double upper_soft_limit = joint_max_[i] - joint_limit_margin_;

        if (q[i] < lower_soft_limit)
        {
            const double ratio = std::min(1., (lower_soft_limit - q[i]) / joint_limit_margin_);
            delta[i] = delta_max_[i] * ratio;
        }
        else if (q[i] > upper_soft_limit)
        {
            const double ratio = std::min(1., (q[i] - upper_soft_limit) / joint_limit_margin_);
            delta[i] = -delta_max_[i] * ratio;
        }
    }

    return delta;
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::calcSingularityAvoidanceDelta(
    const Eigen::Array<double,N_JOINTS,1> &q)
{
    Eigen::Array<double,N_JOINTS,1> delta;
    delta.setZero();

    if (!use_singularity_avoidance_)
    {
        return delta;
    }

    const double sigma_min = calcMinSingularValue(q);
    if (sigma_min >= singularity_sigma_threshold_)
    {
        return delta;
    }

    Eigen::Array<double,N_JOINTS,1> gradient;
    gradient.setZero();

    for (int i = 0; i < N_JOINTS; ++i)
    {
        Eigen::Array<double,N_JOINTS,1> q_plus = q;
        Eigen::Array<double,N_JOINTS,1> q_minus = q;
        q_plus[i] = std::min(q_plus[i] + singularity_gradient_step_, joint_max_[i]);
        q_minus[i] = std::max(q_minus[i] - singularity_gradient_step_, joint_min_[i]);

        const double step = q_plus[i] - q_minus[i];
        if (step > 0.)
        {
            gradient[i] = (calcMinSingularValue(q_plus) - calcMinSingularValue(q_minus)) / step;
        }
    }

    const double max_gradient = gradient.abs().maxCoeff();
    if (max_gradient <= 0.)
    {
        return delta;
    }

    const double activation = std::min(
        1.,
        (singularity_sigma_threshold_ - sigma_min) / singularity_sigma_threshold_);
    delta = gradient / max_gradient * delta_max_ * activation;

    return delta;
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::calcJointLimitTorque(
    const Eigen::Array<double,N_JOINTS,1> &q,
    const Eigen::Array<double,N_JOINTS,1> &dq) const
{
    Eigen::Array<double,N_JOINTS,1> tau;
    tau.setZero();

    for (int i = 0; i < N_JOINTS; ++i)
    {
        const double lower_soft_limit = joint_min_[i] + joint_limit_margin_;
        const double upper_soft_limit = joint_max_[i] - joint_limit_margin_;

        if (q[i] < lower_soft_limit)
        {
            tau[i] = joint_limit_stiffness_[i] * (lower_soft_limit - q[i]) -
                     joint_limit_damping_[i] * dq[i];
        }
        else if (q[i] > upper_soft_limit)
        {
            tau[i] = -joint_limit_stiffness_[i] * (q[i] - upper_soft_limit) -
                     joint_limit_damping_[i] * dq[i];
        }

        tau[i] = std::clamp(
            tau[i],
            -joint_limit_torque_max_[i],
            joint_limit_torque_max_[i]);
    }

    return tau;
}

double TaskSpaceControl::calcMinSingularValue(const Eigen::Array<double,N_JOINTS,1> &q)
{
    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(q);
    Eigen::JacobiSVD<Eigen::Matrix<double,6,N_JOINTS>> svd(J);

    return svd.singularValues().minCoeff();
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::calcSingularityAvoidanceTorque(
    const Eigen::Array<double,N_JOINTS,1> &q,
    const Eigen::Array<double,N_JOINTS,1> &dq)
{
    Eigen::Array<double,N_JOINTS,1> tau;
    tau.setZero();

    if (!use_singularity_avoidance_)
    {
        return tau;
    }

    const double sigma_min = calcMinSingularValue(q);
    if (sigma_min >= singularity_sigma_threshold_)
    {
        return tau;
    }

    Eigen::Array<double,N_JOINTS,1> gradient;
    gradient.setZero();

    for (int i = 0; i < N_JOINTS; ++i)
    {
        Eigen::Array<double,N_JOINTS,1> q_plus = q;
        Eigen::Array<double,N_JOINTS,1> q_minus = q;
        q_plus[i] = std::min(q_plus[i] + singularity_gradient_step_, joint_max_[i]);
        q_minus[i] = std::max(q_minus[i] - singularity_gradient_step_, joint_min_[i]);

        const double step = q_plus[i] - q_minus[i];
        if (step > 0.)
        {
            gradient[i] = (calcMinSingularValue(q_plus) - calcMinSingularValue(q_minus)) / step;
        }
    }

    const double activation = singularity_sigma_threshold_ - sigma_min;
    tau = singularity_stiffness_ * activation * gradient - singularity_damping_ * dq;

    for (int i = 0; i < N_JOINTS; ++i)
    {
        tau[i] = std::clamp(tau[i], -singularity_torque_max_, singularity_torque_max_);
    }

    return tau;
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
