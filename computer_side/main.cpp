#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>

#include "logger/jarraylogger.hpp"

using namespace KUKA_CONTROL;

int main(int argc, char **argv)
{   
    KukaFRIController kuka(KUKA_CONTROL::TORQUE);
    jarray torque = {0, 0, 0, 0, 0, 0, 0};
    kuka.setTargetJointTorque(torque);

    bool flag = true;
    kuka.start();
    jarray current_position;
    jarray current_torque;
    double last_joint_torque = 0;
    constexpr double Kp = 4.9;
    constexpr double Kd = std::sqrt(Kp) * 2;
    const double q_desired = 0.5;
    double q_desired_ = q_desired;
    double q0 = kuka.getMeasuredJointPosition()[5];
    double prev_pos = 0;
    double q_dot = 0;
    bool flag1 = true;

    jarray initial_position = kuka.getMeasuredJointPosition();
    
    LOGGER::JArrayLogger pos_logger("actual_position");
    LOGGER::JArrayLogger torq_logger("actual_torque");

    LOGGER::JArrayLogger commanded_pos_logger("commanded_position");
    LOGGER::JArrayLogger commanded_torq_logger("commanded_torque");

    while (true)
    {
        current_position = kuka.getMeasuredJointPosition();
        current_torque = kuka.getMeasuredJointTorque();

        pos_logger.log(current_position);
        torq_logger.log(current_torque);

        //torque[5] += 0.1;
        q_dot = (current_position[5] - prev_pos) / 0.005;
        
        last_joint_torque += 0.0001;
        //Kp * (q_desired_ - current_position[5]) /* - Kd * q_dot */;
        // if (abs(q_dot) > 0.5){
        //     last_joint_torque = 0;
        //     flag1 = false;
        // }else{
        //     flag1 = true;
        // }
        if(flag1)
            std::cout << std::setprecision(4) << current_position[5] << "\t\t" << q_dot << "\t\t" << current_torque[6] << "\t\t" << last_joint_torque << '\n';
        
        prev_pos = current_position[6];

        // current_position[5]
        // current_position[5] += 0.0001;

        initial_position[6] = current_position[6];

        commanded_pos_logger.log(initial_position);
        commanded_torq_logger.log({0, 0, 0, 0, 0, 0, last_joint_torque});
        kuka.setTargetJointPosition(initial_position);
        kuka.setTargetJointTorque({0, 0, 0, 0, 0, 0, last_joint_torque});
        // kuka.setTarget(torque);
        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    return 0;
}
