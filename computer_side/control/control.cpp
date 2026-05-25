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
    target_torque_ << 0., 0., 0., 0., 0., 0., 0.;
    target_thetta_ = first_thetta;

    // stiffness_ << 25., 25., 25., 20., 15., 10., 8.;
    // damping_ << 2.0, 2.0, 2.0, 1.6, 1.2, 0.8, 0.6;
    stiffness_ << 500, 500, 500, 300, 300, 150, 150;
    damping_ << 30, 30, 30, 20, 20, 10, 10;

    eps_max_ << e_max_, e_max_, e_max_, e_max_, e_max_, e_max_, e_max_;
    eps_min_ << e_min_, e_min_, e_min_, e_min_, e_min_, e_min_, e_min_;
    max_delta_ << max_d_, max_d_, max_d_, max_d_, max_d_, max_d_, max_d_;

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
    const bool same_position = (target_pos - target_pos_).cwiseAbs().maxCoeff() <= target_pos_eps_;
    const bool same_rotation = (target_rot - target_rot_).cwiseAbs().maxCoeff() <= target_rot_eps_;

    if (same_position && same_rotation)
    {
        return ik_state_;
    }

    target_pos_ = target_pos;
    target_rot_ = target_rot;

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
            double step = v_min_ + (v_max_ - v_min_) * ratio;
            vel[i] = std::min(step, std::abs(delta[i])) * sign(delta[i]);
        }
        else
        {
            vel[i] = std::min(v_max_, std::abs(delta[i])) * sign(delta[i]);
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
    Eigen::Array<double,N_JOINTS,1> current_velocity;

    if (!torque_initialized_)
    {
        current_velocity << 0., 0., 0., 0., 0., 0., 0.;
        torque_initialized_ = true;
    }
    else
    {
        current_velocity = (current_thetta - previous_current_thetta_) / time_tick_;
    }

    Eigen::Array<double,N_JOINTS,1> position_error = next_thetta - current_thetta;
    target_torque_ = stiffness_ * position_error - damping_ * current_velocity;

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
