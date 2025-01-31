#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>

#include "logger/jarraylogger.hpp"
#include "control/control_one.hpp"
#include "control/control_full.hpp"
#include "udp/udp_server.hpp"

using namespace KUKA_CONTROL;
using namespace controller_one_joint;
using namespace controller;
using namespace server;

int main(int argc, char **argv)
{   

    // --------------------------- Инициализация сервера

    std::cout << "Hello!\n";

    UDPServer server("127.0.0.1", 8081, "127.0.0.1", 8080);

    std::cout << "Goodbay!\n";

    // --------------------------- Настройки
    
    KukaFRIController kuka(KUKA_CONTROL::TORQUE);

    jarray current_position;
    jarray current_torque;
    jarray torque = {0, 0, 0, 0, 0, 0, 0};

    kuka.setTargetJointTorque(torque);
    kuka.start();

    jarray initial_position = kuka.getJointPosition();
    
    double last_joint_torque = 0;

    // --------------------------- Инициализация конторллера

    Eigen::Array<double,7,1> q_d(initial_position);

    // --------------------------- Инициализация логеров

    LOGGER::JArrayLogger pos_logger("actual_position");

    LOGGER::JArrayLogger commanded_pos_logger("commanded_position");

    server.start();

    while (true)
    {
          
        server.getMsg(q_d);      // Чтение пришедших по UDP данных

        // --------------------------- 

        current_position = kuka.getJointPosition();
        current_torque = kuka.getTorque();

        pos_logger.log(current_position);

        // --------------------------- Расчет управления

        std::cout << q_d.transpose();

        // --------------------------- Задание параметров

        commanded_pos_logger.log({q_d[0],q_d[1],q_d[2],q_d[3],q_d[4],q_d[5],q_d[6]});

        kuka.setTargetJointPosition(initial_position);

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    server.stop();

    return 0;
}