#include "kukafri/kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>
#include <memory>

#include "logger/logger.hpp"
#include "udp/udp_server.hpp"
#include "control/control.hpp"
// #include "control/task_space_control.hpp"
#include "kukafri/helper_functions.hpp"

using namespace KUKA_CONTROL;
using namespace server;

int main(int argc, char **argv)
{   
    // auto mode = KUKA_CONTROL::TORQUE; 
    auto mode = KUKA_CONTROL::JOINT_POSITION; 

    double dt = 0.005;
    bool use_task_space_control = false;

    // --------------------------- Инициализация сервера

    UDPServer<12,25> server("127.0.0.1", 8081, "127.0.0.1", 8080);
    server.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Server started" << std::endl;

    KukaFRIController kuka(mode);
    kuka.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "FRI started" << std::endl;

    // --------------------------- Настройки

    jarray initial_position;
    jarray current_position;

    Eigen::Array<double,12,1> position_msg;
    Eigen::Array<double,6,1> force_msg;
    Eigen::Array<double,25,1> obs_msg;

    Eigen::Vector3d target_pos;
    Eigen::Matrix<double,3,3> target_rot;
    Eigen::Vector3d current_pos;
    Eigen::Matrix<double,3,3> current_rot;

    Eigen::Array<double,7,1> initial_thetta;
    Eigen::Array<double,7,1> current_thetta;
    Eigen::Array<double,7,1> target_thetta;
    
    Eigen::Array<double,7,1> current_torque;
    Eigen::Array<double,7,1> target_torque;
    Eigen::Array<double,7,1> zero_torque; 

    initial_position = kuka.getMeasuredJointPosition(); 
    kuka.setTargetJointPosition(initial_position);
    current_position = initial_position;

    initial_thetta = stdArrayToEigenArray(initial_position);
    target_thetta = initial_thetta;

    target_torque << 0, 0, 0, 0, 0, 0, 0; 
    zero_torque << 0, 0, 0, 0, 0, 0, 0; 
    int state = 0;

    std::unique_ptr<control::IControl> controller;
    if (use_task_space_control)
    {
        // auto task_controller = std::make_unique<control::TaskSpaceControl>("../robots/iiwa.urdf", "iiwa_link_0", "iiwa_link_ee", dt);
        // task_controller->setNullspaceTarget(initial_thetta);
        // controller = std::move(task_controller);
    }
    else
    {
        controller = std::make_unique<control::Control>(initial_thetta, dt);
    }

    controller->updateCurrentState(initial_thetta, target_torque);
    target_pos = controller->getCurrentPosition();
    target_rot = controller->getCurrentRotation();
    state = controller->updateTarget(target_pos, target_rot);

    // trajectory::waitConnection();

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // logger::FileLogger log = logger::FileLogger("Logs.txt");
    // log.start();
    // Eigen::Array<double,9,1> log_data;
    // std::chrono::steady_clock::time_point init_time_ = std::chrono::steady_clock::now();

    // Eigen::Array<double,7,1> mid_point;
    // mid_point << 0, 45*M_PI/180, 0, -45*M_PI/180, 0, 30*M_PI/180, 0;

    // int i = 0;  
    // int n = 0;
    // int T = 750;

    while (true)
    {   

        if (server.getMsg(position_msg))  // Чтение пришедших по UDP данных
        {
            target_pos[0] += position_msg[0];
            target_pos[1] += position_msg[1];
            target_pos[2] += position_msg[2];

            target_rot << position_msg[3], position_msg[4], position_msg[5],
                          position_msg[6], position_msg[7], position_msg[8],
                          position_msg[9], position_msg[10], position_msg[11];

            state = controller->updateTarget(target_pos, target_rot);
            target_thetta = controller->getTargetThetta();
        };

        // if (n < 5) target_thetta << 0.23*sin(2*i*M_PI/T) + mid_point[0], mid_point[1], mid_point[2], 0.15*sin(4*i*M_PI/T)+mid_point[3], mid_point[4], 0.1*cos(4*i*M_PI/T)+mid_point[5], mid_point[6];
        // if (i++ > T) 
        // {i = 0; n += 1;} 

        // ========================================================================================
        
        current_thetta = stdArrayToEigenArray(kuka.getMeasuredJointPosition());
        current_torque = stdArrayToEigenArray(kuka.getExternalJointTorque());
        controller->updateCurrentState(current_thetta, current_torque);

        if (mode == KUKA_CONTROL::JOINT_POSITION)
        {
            kuka.setTargetJointPosition(eigenArrayToStdArray(target_thetta));
        }
        else if (mode == KUKA_CONTROL::TORQUE)
        {
            target_torque = controller->getTorque();
            kuka.setTargetJointPosition(eigenArrayToStdArray(current_thetta));
            kuka.setTargetJointTorque(eigenArrayToStdArray(target_torque));
        }

        std::cout << "State: " << state << std::endl;
        std::cout << "Target: " << target_thetta.transpose()*180/M_PI << std::endl;
        std::cout << "Current: " << current_thetta.transpose()*180/M_PI << std::endl;
        if (mode == KUKA_CONTROL::TORQUE)
        {
            std::cout << "Target torque: " << target_torque.transpose() << std::endl;
        }
        std::cout << "Torque: " << current_torque.transpose() << std::endl;
        std::cout << "=================================================" << std::endl;
        
        // log_data << 0, (double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init_time_).count()), target_thetta[0], target_thetta[1], target_thetta[2], target_thetta[3], target_thetta[4], target_thetta[5], target_thetta[6]; 
        // log.setData(log_data);
        // log_data << 1, (double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init_time_).count()), current_thetta[0], current_thetta[1], current_thetta[2], current_thetta[3], current_thetta[4], current_thetta[5], current_thetta[6]; 
        // log.setData(log_data);
        // log_data << 2, (double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init_time_).count()), target_torque[0], target_torque[1], target_torque[2], target_torque[3], target_torque[4], target_torque[5], target_torque[6]; 
        // log.setData(log_data);
        // log_data << 3, (double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - init_time_).count()), current_torque[0], current_torque[1], current_torque[2], current_torque[3], current_torque[4], current_torque[5], current_torque[6]; 
        // log.setData(log_data);

        // ========================================================================================

        current_pos = controller->getCurrentPosition();
        current_rot = controller->getCurrentRotation();
        force_msg = controller->getForce();

        obs_msg <<  current_thetta[0], current_thetta[1], current_thetta[2], current_thetta[3], current_thetta[4], current_thetta[5], current_thetta[6], 
                    current_pos[0], current_pos[1], current_pos[2],
                    current_rot(0,0), current_rot(0,1), current_rot(0,2),
                    current_rot(1,0), current_rot(1,1), current_rot(1,2),
                    current_rot(2,0), current_rot(2,1), current_rot(2,2),
                    force_msg[0], force_msg[1], force_msg[2], force_msg[3], force_msg[4], force_msg[5];
        server.setMsg(obs_msg);

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    kuka.stop();
    server.stop();

    return 0;
}
