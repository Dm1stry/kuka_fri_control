#include "control.hpp"

using namespace control;

Control::Control(const Eigen::Array<double,7,1> &first_thetta, double& time_tick, const std::string &urdf_name):
time_tick_(time_tick),
solver_(urdf_name)
{
    points_.push_back(first_thetta);

    virtual_thetta_ = first_thetta;
    next_thetta_ = first_thetta;
    current_thetta_ = first_thetta;
    current_torque_ << 0., 0., 0., 0., 0., 0., 0.;
    previous_current_thetta_ = first_thetta;
    previous_target_thetta_ = first_thetta;
    filtered_current_velocity_ << 0., 0., 0., 0., 0., 0., 0.;
    target_torque_ << 0., 0., 0., 0., 0., 0., 0.;
    target_thetta_ = first_thetta;

    // stiffness_ << 25., 25., 25., 20., 15., 10., 8.;
    // damping_ << 2.0, 2.0, 2.0, 1.6, 1.2, 0.8, 0.6;
    stiffness_ << 300, 300, 300, 150, 150, 75, 75;
    damping_ << 30, 30, 30, 15, 15, 7.5, 7.5;
    cabinet_stiffness_ << 500., 500., 500., 300., 300., 150., 150.;
    torque_rate_limit_ << 3.0, 3.0, 3.0, 3.0, 3.0, 2.0, 2.0;

    eps_max_ << e_max_, e_max_, e_max_, e_max_, e_max_, e_max_, e_max_;
    eps_min_ << e_min_, e_min_, e_min_, e_min_, e_min_, e_min_, e_min_;
    max_delta_ << max_d_, max_d_, max_d_, max_d_, max_d_, max_d_, max_d_;
    delta_max_ = max_joint_velocity_ * time_tick_; // * cabinet_stiffness_ / cabinet_stiffness_.maxCoeff();
    delta_min_ = delta_max_ * min_delta_ratio_;

    solver_.setQ(first_thetta);
    solver_.FK();
    current_pos_ = solver_.getPositionVector();
    current_rot_ = solver_.getRotationMatrix();
    target_pos_ = current_pos_;
    target_rot_ = current_rot_;
    force_ << 0., 0., 0., 0., 0., 0.;
}

// =======================================================================

bool Control::push(const Eigen::Array<double,7,1> &thetta)
{
    points_.push_back(thetta);
    return true;
}

bool Control::pop(Eigen::Array<double,7,1> &thetta)
{
    if (points_.size() == 0)
    {
        return false;
    }
    thetta = points_.front();
    points_.pop_front();

    return true;
}

size_t Control::size()
{
    return points_.size();
}

// =======================================================================

int Control::updateTarget(const Eigen::Vector3d &target_pos, const Eigen::Matrix<double,3,3> &target_rot)
{
    Eigen::Vector3d filtered_target_pos;
    Eigen::Matrix3d filtered_target_rot;

    if (!target_filter_initialized_)
    {
        filtered_target_pos = target_pos;
        filtered_target_rot = target_rot;
        target_filter_initialized_ = true;
    }
    else
    {
        const double new_target_weight = 1. - target_filter_alpha_;
        filtered_target_pos = target_filter_alpha_ * target_pos_ + new_target_weight * target_pos;

        Eigen::Quaterniond current_target(target_rot_);
        Eigen::Quaterniond new_target(target_rot);
        current_target.normalize();
        new_target.normalize();

        if (current_target.dot(new_target) < 0.)
        {
            new_target.coeffs() *= -1.;
        }

        filtered_target_rot = current_target.slerp(new_target_weight, new_target).normalized().toRotationMatrix();
    }

    const bool same_position = (filtered_target_pos - target_pos_).cwiseAbs().maxCoeff() <= target_pos_eps_;
    const bool same_rotation = (filtered_target_rot - target_rot_).cwiseAbs().maxCoeff() <= target_rot_eps_;

    if (same_position && same_rotation)
    {
        return ik_state_;
    }

    target_pos_ = filtered_target_pos;
    target_rot_ = filtered_target_rot;

    solver_.setPositionVector(target_pos_);
    solver_.setRotationMatrix(target_rot_);
    ik_state_ = solver_.IK_minQ();
    target_thetta_ = solver_.getQ();

    return ik_state_;
}

void Control::updateCurrentState(const Eigen::Array<double,N_JOINTS,1> &current_thetta, const Eigen::Array<double,N_JOINTS,1> &current_torque)
{
    current_thetta_ = current_thetta;
    current_torque_ = current_torque;

    solver_.setQ(current_thetta);
    solver_.FK();
    current_pos_ = solver_.getPositionVector();
    current_rot_ = solver_.getRotationMatrix();
    force_ = solver_.getForce(current_thetta, current_torque);
}

Eigen::Array<double,N_JOINTS,1> Control::getTorque()
{
    next_thetta_ = getNextPoint(target_thetta_, current_thetta_);
    return getTorque(next_thetta_, current_thetta_);
}

Eigen::Array<double,N_JOINTS,1> Control::getNextPoint()
{
    next_thetta_ = getNextPoint(target_thetta_, current_thetta_);
    return next_thetta_;
}

Eigen::Array<double,N_JOINTS,1> Control::getTargetThetta() const
{
    return target_thetta_;
}

Eigen::Vector3d Control::getCurrentPosition() const
{
    return current_pos_;
}

Eigen::Matrix<double,3,3> Control::getCurrentRotation() const
{
    return current_rot_;
}

Eigen::Matrix<double,6,1> Control::getForce() const
{
    return force_;
}

int Control::getState() const
{
    return ik_state_;
}

int Control::getIKState() const
{
    return ik_state_;
}

// =======================================================================

Eigen::Array<double,N_JOINTS,1> Control::getDelta(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta)
{
    Eigen::Array<double,N_JOINTS,1> delta = next_thetta - current_thetta;
    Eigen::Array<double,N_JOINTS,1> vel;

    for(int i = 0; i < N_JOINTS; ++i)
    {
        if ((std::abs(delta[i]) <= eps_min_[i]))
        {
            vel[i] = 0.;
        }
        else if(std::abs(delta[i]) < eps_max_[i])
        {
            double ratio = (std::abs(delta[i]) - eps_min_[i]) / (eps_max_[i] - eps_min_[i]);
            ratio = ratio * ratio * (3. - 2. * ratio);
            double step = delta_max_[i] * ratio;
            vel[i] = std::min(step, std::abs(delta[i])) * sign(delta[i]);
        }
        else
        {
            vel[i] = std::min(delta_max_[i], std::abs(delta[i])) * sign(delta[i]);
        }

    }

    return vel;
}

Eigen::Array<double,N_JOINTS,1>& Control::getNextPoint(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta)
{
    
    virtual_thetta_ = virtual_thetta_ + this->getDelta(next_thetta, virtual_thetta_);

    return virtual_thetta_;
}

bool Control::getDone()
{
    return done_;
}

Eigen::Array<double,N_JOINTS,1>& Control::getTorque(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta)
{
    Eigen::Array<double,N_JOINTS,1> raw_current_velocity;

    if (!torque_initialized_)
    {
        raw_current_velocity << 0., 0., 0., 0., 0., 0., 0.;
        filtered_current_velocity_ = raw_current_velocity;
        torque_initialized_ = true;
    }
    else
    {
        raw_current_velocity = (current_thetta - previous_current_thetta_) / time_tick_;
        filtered_current_velocity_ =
            velocity_filter_alpha_ * filtered_current_velocity_ +
            (1. - velocity_filter_alpha_) * raw_current_velocity;
    }

    Eigen::Array<double,N_JOINTS,1> position_error = next_thetta - current_thetta;
    Eigen::Array<double,N_JOINTS,1> desired_torque =
        stiffness_ * position_error - damping_ * filtered_current_velocity_;

    for(int i = 0; i < N_JOINTS; ++i)
    {
        const double torque_delta = desired_torque[i] - target_torque_[i];
        const double limited_delta = std::clamp(
            torque_delta,
            -torque_rate_limit_[i],
            torque_rate_limit_[i]);
        target_torque_[i] += limited_delta;
    }

    // for(int i = 0; i < N_JOINTS; ++i)
    // {
    //     if (std::abs(position_error[i]) <= eps_min_[i])
    //     {
    //         target_torque_[i] = 0.;
    //     }
    // }

    previous_target_thetta_ = next_thetta;
    previous_current_thetta_ = current_thetta;

    return target_torque_;
}

// =======================================================================

bool control::eigenArrayEqual(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &eps)
{
    for(int i = 0; i < N_JOINTS; ++i)
    {
        if(std::abs(arr1[i]-arr2[i]) > eps[i]) 
        {
            return false;
        }
    }
    return true;
}

bool control::eigenArrayDiff(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &diff)
{
    for(int i = 0; i < N_JOINTS; ++i)
    {
        if(std::abs(arr1[i]-arr2[i]) > diff[i]) 
        {
            return true;
        }
    }
    return false;
}

int control::sign(double a)
{
    if (a > 0)
    {
        return 1;
    }
    else if (a < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void control::waitConnection()
{
    std::cout << "alla" << std::endl;
    const char* name = "/my_shm2";
    int shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return;
    }

    ftruncate(shm_fd, sizeof(bool));

    bool* ptr = (bool*)mmap(0, sizeof(bool), PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // const char* message = "Временно";
    // std::memcpy(ptr, message, strlen(message) + 1);
    *ptr = false;

    ptr = (bool*)mmap(0, sizeof(bool), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        return;
    }

    std::cout << "Ожидание..." << std::endl;

    while(1)
    {
        // std::cout << *(static_cast<bool*>(ptr)) << std::endl;
        if (*(static_cast<bool*>(ptr)))
        {
            break;
        }
    }

    std::cout << "Начало" << std::endl;

    munmap(ptr, sizeof(bool));
    close(shm_fd);
    shm_unlink(name);
}
