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
    current_position_.setZero();
    current_rotation_.setIdentity();
    task_error_.setZero();
    force_.setZero();
    target_wrench_.setZero();

    stiffness_ << 500., 500., 500., 80., 80., 80.;
    damping_ << 45., 45., 45., 8., 8., 8.;

    current_q_.setZero();
    current_dq_.setZero();
    previous_q_.setZero();
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

    const Eigen::Matrix<double,6,N_JOINTS> J = calcJacobian(current_q_);
    force_ = J.transpose().fullPivHouseholderQr().solve(current_torque.matrix());
}

Eigen::Array<double,N_JOINTS,1> TaskSpaceControl::getTorque()
{
    target_torque_ = getTorque(current_q_, current_dq_);
    return target_torque_;
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

    std::cout << task_error_ << std::endl;

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
    return q_ref_;
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
    Eigen::Matrix<double,6,1> error;
    error.head<3>() = target_position_ - current_position;

    Eigen::AngleAxisd angle_axis(target_rotation_ * current_rotation.transpose());
    error.tail<3>() = angle_axis.angle() * angle_axis.axis();

    return error;
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
