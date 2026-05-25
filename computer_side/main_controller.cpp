#include "main_controller.hpp"

KukaController::KukaController(KUKA_CONTROL::control_mode mode, const std::string &urdf_name, bool use_task_space):
mode_(mode),
use_task_space_(use_task_space),
kuka_(mode),
urdf_name_(urdf_name)
{
    dt_ = 0.005;
    state_ = 0;
    target_torque_ << 0, 0, 0, 0, 0, 0, 0; 
    zero_torque_ << 0, 0, 0, 0, 0, 0, 0; 
    obs_msg_.setZero();
}

KukaController::~KukaController()
{
    stop();
}

void KukaController::start()
{
    if (loop_thread_.joinable())
    {
        return;
    }

    kuka_.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "FRI started" << std::endl;

    auto initial_position = kuka_.getMeasuredJointPosition(); 
    kuka_.setTargetJointPosition(initial_position);

    initial_thetta_ = stdArrayToEigenArray(initial_position);
    target_thetta_ = initial_thetta_;

    if (use_task_space_)
    {
        auto task_controller = std::make_unique<control::TaskSpaceControl>(urdf_name_, "iiwa_link_0", "iiwa_link_ee", dt_);
        task_controller->setNullspaceTarget(initial_thetta_);
        controller_ = std::move(task_controller);
    }

    if (!controller_)
    {
        controller_ = std::make_unique<control::Control>(initial_thetta_, dt_, urdf_name_);
    }

    controller_->updateCurrentState(initial_thetta_, target_torque_);
    target_pos_ = controller_->getCurrentPosition();
    target_rot_ = controller_->getCurrentRotation();
    state_ = controller_->updateTarget(target_pos_, target_rot_);
    target_thetta_ = controller_->getTargetThetta();

    current_thetta_ = initial_thetta_;
    current_torque_ = target_torque_;
    current_pos_ = target_pos_;
    current_rot_ = target_rot_;
    force_msg_ = controller_->getForce();
    obs_msg_ <<  current_thetta_[0], current_thetta_[1], current_thetta_[2], current_thetta_[3], current_thetta_[4], current_thetta_[5], current_thetta_[6],
                current_pos_[0], current_pos_[1], current_pos_[2],
                current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
                current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
                current_rot_(2,0), current_rot_(2,1), current_rot_(2,2),
                force_msg_[0], force_msg_[1], force_msg_[2], force_msg_[3], force_msg_[4], force_msg_[5];

    loop_thread_ = std::jthread([this](std::stop_token stop_token) {
        loop(stop_token);
    });
}

void KukaController::stop()
{
    if (loop_thread_.joinable())
    {
        loop_thread_.request_stop();
        loop_thread_.join();
    }

    kuka_.stop();
}

void KukaController::loop(std::stop_token stop_token)
{   
    auto init = std::chrono::steady_clock::now();
    while (!stop_token.stop_requested())
    {   
        
        current_thetta_ = stdArrayToEigenArray(kuka_.getMeasuredJointPosition());
        current_torque_ = stdArrayToEigenArray(kuka_.getExternalJointTorque());

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // init = std::chrono::steady_clock::now();
            controller_->updateCurrentState(current_thetta_, current_torque_);

            if (mode_ == KUKA_CONTROL::JOINT_POSITION)
            {
                target_thetta_ = controller_->getNextPoint();
                kuka_.setTargetJointPosition(eigenArrayToStdArray(target_thetta_));
            }
            else if (mode_ == KUKA_CONTROL::TORQUE)
            {
                target_torque_ = controller_->getTorque();
                kuka_.setTargetJointPosition(eigenArrayToStdArray(current_thetta_));
                kuka_.setTargetJointTorque(eigenArrayToStdArray(target_torque_));
            }

            // std::cout << "State: " << state_ << std::endl;
            // std::cout << "Target: " << target_thetta_.transpose()*180/M_PI << std::endl;
            // std::cout << "Current: " << current_thetta_.transpose()*180/M_PI << std::endl;
            // if (mode_ == KUKA_CONTROL::TORQUE)
            // {
            //     std::cout << "Target torque: " << target_torque_.transpose() << std::endl;
            // }
            // std::cout << "Torque: " << current_torque_.transpose() << std::endl;
            // std::cout << "=================================================" << std::endl;

            current_pos_ = controller_->getCurrentPosition();
            current_rot_ = controller_->getCurrentRotation();
            force_msg_ = controller_->getForce();
            obs_msg_ <<  current_thetta_[0], current_thetta_[1], current_thetta_[2], current_thetta_[3], current_thetta_[4], current_thetta_[5], current_thetta_[6], 
                        current_pos_[0], current_pos_[1], current_pos_[2],
                        current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
                        current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
                        current_rot_(2,0), current_rot_(2,1), current_rot_(2,2),
                        force_msg_[0], force_msg_[1], force_msg_[2], force_msg_[3], force_msg_[4], force_msg_[5];
        }

        // std::cout << "loop: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init).count() << "\n";
        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }
}

Eigen::Array<double,25,1> KukaController::getObservation()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return obs_msg_;
}

void KukaController::setTarget(const Eigen::Vector3d& target_position, const Eigen::Matrix<double,3,3>& target_rotation)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // auto init = std::chrono::steady_clock::now();
    target_pos_ = target_position;
    target_rot_ = target_rotation;
    state_ = controller_->updateTarget(target_pos_, target_rot_);
    target_thetta_ = controller_->getTargetThetta();
    // std::cout << "set: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init).count() << "\n";
}
