#include "kukafri/kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <fstream>

#include "logger/jarraylogger.hpp"
#include "udp/udp_server.hpp"
#include "planer/trajectory.hpp"
#include "kukafri/helper_functions.hpp"

using namespace KUKA_CONTROL;
using namespace server;

int main(int argc, char **argv)
{   

    // --------------------------- Инициализация сервера

    std::cout << "Hello!\n";

    UDPServer server("127.0.0.1", 8081, "127.0.0.1", 8080);

    std::cout << "Goodbye!\n";

    // --------------------------- Настройки
    
    KukaFRIController kuka(KUKA_CONTROL::JOINT_POSITION);

    jarray current_position;
    jarray initial_position;

    Eigen::Array<double,7,1> current_point;
    Eigen::Array<double,7,1> initial_point;
    Eigen::Array<double,7,1> temp;
    Eigen::Array<double,7,1> delta;
    Eigen::Array<double,7,1> msg_thetta; 

    Eigen::Array<double,7,1> current_torque; 
    Eigen::Array<double,14,1> msg_torque; 

    const double e = 0.1*M_PI/180;
    const double df = 5*M_PI/180;

    Eigen::Array<double,7,1> eps;
    Eigen::Array<double,7,1> diff;

    eps << e, e, e, e, e, e, e;
    diff << df, df, df, df, df, df, df;

    bool done = true;

    // trajectory::waitConnection();

    kuka.start();

    initial_position = kuka.getMeasuredJointPosition();
    kuka.setTargetJointPosition(initial_position);

    current_position = initial_position;

    initial_point = stdArrayToEigenArray(initial_position);
    Eigen::Array<double,7,1> next_point = initial_point;
    temp = initial_point;

    trajectory::Trajectory planer(initial_point);
    
    // --------------------------- Инициализация логеров

    // LOGGER::JArrayLogger pos_logger("actual_position");
    // LOGGER::JArrayLogger commanded_pos_logger("commanded_position");
    // LOGGER::JArrayLogger delta_pos_logger("delta_position");

    server.start();

    std::cout << "Старт" << std::endl;

    while (true)
    {
        if (server.getMsg(next_point))
        {
            // current_position = {q_d[0],q_d[1],q_d[2],q_d[3],q_d[4],q_d[5],q_d[6]};
            // previous_position = current_position;
            // std::cout << q_d.transpose() << std::endl;
            // planer.push(msg_thetta);          
            std::cout << next_point.transpose() << std::endl;

        };      // Чтение пришедших по UDP данных

        // ========================================================================================

        current_point = stdArrayToEigenArray(kuka.getMeasuredJointPosition());

        std::cout << "Current: " << current_point.transpose()*180/M_PI << std::endl;

        // std::cout << "Время: " << ((double)(clock() - t))/CLOCKS_PER_SEC*1000 << std::endl;

        if (false)
        {
            done = !(planer.pop(next_point));
            // std::cout << "POP" << done << std::endl;
        }
        else
        {
            // temp = stdArrayToEigenArray(kuka.getCommandedJointPosition());

            if (!trajectory::eigenArrayDiff(temp,current_point,diff))
            {
                delta = planer.getDelta(next_point, temp);
                temp = temp + delta;
            }

            // std::cout << "Commanded: " << temp.transpose()*180/M_PI << std::endl;

            kuka.setTargetJointPosition(eigenArrayToStdArray(temp));

            if (trajectory::eigenArrayEqual(temp,next_point,eps))
            {
                done = true;
                // std::cout << "============================DONE============================" << std::endl;
            }
        }
        
        // ========================================================================================

        current_torque = stdArrayToEigenArray(kuka.getExternalJointTorque());

        std::cout << "Torque: " << current_torque.transpose() << std::endl;

        msg_torque << current_point, current_torque;

        server.setMsg(msg_torque);

        // commanded_pos_logger.log(eigenArrayToStdArray(temp));
        // pos_logger.log(eigenArrayToStdArray(current_point));
        // delta_pos_logger.log(eigenArrayToStdArray(delta));

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    server.stop();

    return 0;
}