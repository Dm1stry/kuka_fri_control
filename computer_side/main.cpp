#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>

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
    double q0 = kuka.getJointPosition()[5];
    double prev_pos = 0;
    double q_dot = 0;
    bool flag1 = true;

    jarray initial_position = kuka.getJointPosition();

    while (true)
    {
        current_position = kuka.getJointPosition();
        current_torque = kuka.getTorque();
        
        //torque[5] += 0.1;
        q_dot = (current_position[5] - prev_pos) / 0.005;
        
        last_joint_torque = Kp * (q_desired_ - current_position[5]) /* - Kd * q_dot */;
        if (abs(q_dot) > 0.5){
            last_joint_torque = 0;
            flag1 = false;
        }else{
            flag1 = true;
        }
        if(flag1)
            std::cout << std::setprecision(4) << current_position[5] << "\t\t" << q_dot << "\t\t" << current_torque[5] << "\t\t" << last_joint_torque << '\n';
        
        prev_pos = current_position[5];

        // current_position[5]
        // current_position[5] += 0.0001;

        initial_position[5] = current_position[5];

        kuka.setTargetJointPosition(initial_position);
        kuka.setTargetJointTorque({0, 0, 0, 0, 0, last_joint_torque, 0});
        // kuka.setTarget(torque);
        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    return 0;
}
