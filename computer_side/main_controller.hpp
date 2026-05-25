#ifndef MAIN_CONTROLLER
#define MAIN_CONTROLLER

#include "kukafri/kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <stop_token>

#include "logger/logger.hpp"
#include "udp/udp_server.hpp"
#include "control/control.hpp"
// #include "control/task_space_control.hpp"
#include "kukafri/helper_functions.hpp"
#include "kukafri/customlbrclient.hpp"


class KukaController
{
public:
    KukaController(KUKA_CONTROL::control_mode mode = KUKA_CONTROL::JOINT_POSITION, const std::string &urdf_name = "../robots/iiwa.urdf", bool use_task_space = false);
    ~KukaController();

    void start();
    void stop();

    Eigen::Array<double,25,1> getObservation();
    void setTarget(const Eigen::Vector3d& target_position, const Eigen::Matrix<double,3,3>& target_rotation);
    
private:
    void loop(std::stop_token stop_token);

    KUKA_CONTROL::KukaFRIController kuka_;
    std::unique_ptr<control::IControl> controller_;
    std::jthread loop_thread_;
    std::mutex state_mutex_;

    KUKA_CONTROL::control_mode mode_;
    bool use_task_space_;
    double dt_;
    const std::string urdf_name_;

    int state_;

    Eigen::Vector3d target_pos_;
    Eigen::Matrix<double,3,3> target_rot_;
    Eigen::Vector3d current_pos_;
    Eigen::Matrix<double,3,3> current_rot_;

    Eigen::Array<double,7,1> initial_thetta_;
    Eigen::Array<double,7,1> current_thetta_;
    Eigen::Array<double,7,1> target_thetta_;
    
    Eigen::Array<double,7,1> current_torque_;
    Eigen::Array<double,7,1> target_torque_;
    Eigen::Array<double,7,1> zero_torque_;

    Eigen::Array<double,6,1> force_msg_;
    Eigen::Array<double,25,1> obs_msg_;
    
};

#endif
