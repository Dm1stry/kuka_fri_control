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
    Eigen::Array<double,7,1> initial_point;
    Eigen::Array<double,7,1> temp;

    kuka.start();

    initial_position = kuka.getMeasuredJointPosition();
    current_position = initial_position;

    initial_point = stdArrayToEigenArray(initial_position);
    Eigen::Array<double,7,1> next_point = initial_point + 20*M_PI/180;

    trajectory::Trajectory planer(initial_point);
    planer.push(next_point);
    planer.push(initial_point);

    
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

        temp = planer.calcTransferedPoint();

        std::cout << "Commanded: " << temp.transpose()*180/M_PI << std::endl;

        kuka.setTargetJointPosition(eigenArrayToStdArray(temp));

        current_point = stdArrayToEigenArray(kuka.getMeasuredJointPosition());

        if (planer.getDone())
        {
            planer.synchPosition(current_point);
        }

        std::cout << "Current: " << current_point.transpose()*180/M_PI << std::endl;
        
        // ========================================================================================

        commanded_pos_logger.log(eigenArrayToStdArray(temp));
        pos_logger.log(eigenArrayToStdArray(current_point));

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    server.stop();

    return 0;
}