#include "kukafri/kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>

#include "logger/jarraylogger.hpp"
#include "udp/udp_server.hpp"
#include "control/control.hpp"
#include "kukafri/helper_functions.hpp"
#include "ik/drake_kinematic.hpp"

using namespace KUKA_CONTROL;
using namespace server;

int main(int argc, char **argv)
{   
    auto mode = KUKA_CONTROL::JOINT_POSITION; 

    // --------------------------- Инициализация сервера

    UDPServer<12,6> server("127.0.0.1", 8081, "127.0.0.1", 8080);
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

    Eigen::Vector3d target_pos;
    Eigen::Matrix<double,3,3> target_rot;
    Eigen::Vector3d current_pos;
    Eigen::Matrix<double,3,3> current_rot;

    Eigen::Array<double,7,1> initial_thetta;
    Eigen::Array<double,7,1> current_thetta;
    Eigen::Array<double,7,1> target_thetta;
    Eigen::Array<double,7,1> next_thetta;
    
    Eigen::Array<double,7,1> current_torque; 
    Eigen::Array<double,14,1> msg_torque; 

    initial_position = kuka.getMeasuredJointPosition(); 
    kuka.setTargetJointPosition(initial_position);
    current_position = initial_position;

    initial_thetta = stdArrayToEigenArray(initial_position);
    target_thetta = initial_thetta;
    next_thetta = initial_thetta;

    control::Control control(initial_thetta);

    int state = 0;
    iiwa_kinematics::DrakeKinematic solver("../robots/iiwa.urdf");
    solver.setQ(initial_thetta);
    solver.FK();
    current_pos = solver.getPositionVector();
    current_rot = solver.getRotationMatrix();
    target_pos = current_pos;
    target_rot = current_rot;
    // trajectory::waitConnection();

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

            solver.setPositionVector(target_pos);
            solver.setRotationMatrix(target_rot);
            state = solver.IK_minQ();
            target_thetta = solver.getQ();
            
        };

        // ========================================================================================
        
        current_thetta = stdArrayToEigenArray(kuka.getMeasuredJointPosition());
        current_torque = stdArrayToEigenArray(kuka.getExternalJointTorque());

        std::cout << "State: " << state << std::endl;
        std::cout << "Target: " << target_thetta.transpose()*180/M_PI << std::endl;
        std::cout << "Current: " << current_thetta.transpose()*180/M_PI << std::endl;
        std::cout << "Torque: " << current_torque.transpose() << std::endl;
        std::cout << "=================================================" << std::endl;

        next_thetta = control.getNextPoint(target_thetta, current_thetta);

        kuka.setTargetJointPosition(eigenArrayToStdArray(next_thetta));

        // ========================================================================================

        // msg_torque << current_thetta, current_torque;
        force_msg = solver.getForce(current_thetta, current_torque);
        server.setMsg(force_msg);

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    kuka.stop();
    server.stop();

    return 0;
}