#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>

#include "logger/jarraylogger.hpp"
// #include "control/control_one.hpp"
// #include "control/control_full.hpp"
#include "udp/udp_server.hpp"
#include "planer/trajectory.hpp"
#include "helper_functions.hpp"

using namespace KUKA_CONTROL;
// using namespace controller_one_joint;
// using namespace controller;
using namespace server;

int main(int argc, char **argv)
{   

    // --------------------------- Инициализация сервера

    std::cout << "Hello!\n";

    UDPServer server("127.0.0.1", 8081, "127.0.0.1", 8080);

    std::cout << "Goodbay!\n";

    // --------------------------- Настройки
    
    KukaFRIController kuka(KUKA_CONTROL::JOINT_POSITION);

    jarray current_position;
    jarray initial_position;

    Eigen::Array<double,7,1> current_point;
    Eigen::Array<double,7,1> initial_q;
    Eigen::Array<double,7,1> eps;
    Eigen::Array<double,7,1> temp;
    double e = 1;
    eps << e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180;

    // jarray current_torque;
    // jarray torque = {0, 0, 0, 0, 0, 0, 0};
    // kuka.setTargetJointTorque(torque);

    kuka.start();

    initial_position = kuka.getMeasuredJointPosition();
    current_position = initial_position;

    initial_q = stdArrayToEigenArray(initial_position);
    Eigen::Array<double,7,1> next_point = initial_q + 30*M_PI/180;
    temp = initial_q + 2*M_PI/180;

    bool done = true;
    trajectory::Trajectory planer(initial_q);
    planer.push(next_point);
    
    // --------------------------- Инициализация логеров

    LOGGER::JArrayLogger pos_logger("actual_position");
    LOGGER::JArrayLogger commanded_pos_logger("commanded_position");

    server.start();

    while (true)
    {
          
        // if (server.getMsg(q_d))
        // {
        //     current_position = {q_d[0],q_d[1],q_d[2],q_d[3],q_d[4],q_d[5],q_d[6]};
        //     // previous_position = current_position;
        //     // std::cout << q_d.transpose() << std::endl;

        // };      // Чтение пришедших по UDP данных

        // ========================================================================================

        if (done)
        {
            planer.pop(next_point);
            done = false;
            kuka.setTargetJointPosition(eigenArrayToStdArray(next_point));
            std::cout << planer.size() << "Done\n";
            std::cout << "Next: " << next_point.transpose()*180/M_PI << std::endl;
        }
        else
        {   
            current_point = stdArrayToEigenArray(kuka.getMeasuredJointPosition());
            
            std::cout << "Next: " << next_point.transpose()*180/M_PI << std::endl;
            std::cout << "Current: " << current_point.transpose()*180/M_PI << std::endl;

            if (trajectory::eigenArrayEqual(current_point, next_point, eps))
            {
                done = true;
            }

            kuka.setTargetJointPosition(eigenArrayToStdArray(next_point));
            
            // std::cout << temp[0]*180/M_PI << "\t"<< temp[1]*180/M_PI << "\t"<< temp[2]*180/M_PI << "\t"<< temp[3]*180/M_PI << "\t"<< temp[4]*180/M_PI << "\t"<< temp[5]*180/M_PI << "\t"<< temp[6]*180/M_PI<< "\t" << std::endl;
        }
        
        // ========================================================================================

        // current_torque = kuka.getTorque();

        commanded_pos_logger.log(eigenArrayToStdArray(next_point));
        pos_logger.log(eigenArrayToStdArray(current_point));

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    server.stop();

    return 0;
}