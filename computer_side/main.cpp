#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>

#include "logger/jarraylogger.hpp"
#include "control/control_one.hpp"
#include "udp_server/udp_server.hpp"

using namespace KUKA_CONTROL;
using namespace controller_one_joint;
using namespace server;

int main(int argc, char **argv)
{   

    // --------------------------- Инициализация сервера

    std::cout << "Hello!\n";

    UDPServer server("127.0.0.1", 8080, "127.0.0.1", 8081);

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

    ControlOne contrololo(0, 0, 1.5, 0.005);
    contrololo.setPreviousPos(initial_position[6]);

    double q_d = 0;     // Переменная для считывания приходящих по UDP значений

    // --------------------------- Инициализация логеров

    LOGGER::JArrayLogger pos_logger("actual_position");
    LOGGER::JArrayLogger torq_logger("actual_torque");

    LOGGER::JArrayLogger commanded_pos_logger("commanded_position");
    LOGGER::JArrayLogger commanded_torq_logger("commanded_torque");

    server.start();

    while (true)
    {

        // if (server.getNumber(q_d))
        // {
        //     std::cout << q_d << std::endl;
        // }

        // std::cout << "*******" << std::endl;

          
        server.getNumber(q_d);      // Чтение пришедших по UDP данных

        // --------------------------- 

        current_position = kuka.getJointPosition();
        current_torque = kuka.getTorque();

        pos_logger.log(current_position);
        torq_logger.log(current_torque);

        initial_position[6] = current_position[6];

        // --------------------------- Расчет управления

        last_joint_torque = contrololo.calcTorque(current_position[6], q_d);

        std::cout << current_position[6]*180/M_PI << "\t" << q_d*180/M_PI << "\t" << last_joint_torque << std::endl;
        
        // --------------------------- Задание параметров

        commanded_pos_logger.log(initial_position);
        commanded_torq_logger.log({0, 0, 0, 0, 0, 0, last_joint_torque});

        kuka.setTargetJointPosition(initial_position);
        kuka.setTargetJointTorque({0, 0, 0, 0, 0, 0, last_joint_torque});
        // kuka.setTarget(torque);

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    server.stop();

    return 0;
}