#ifndef KUKA_FRI_CONTROLLER_HPP
#define KUKA_FRI_CONTROLLER_HPP

#include <stdint.h>
#include <string>
#include <thread>
#include <functional>
#include <atomic>

#include <FRI/friUdpConnection.h>
#include <FRI/friClientApplication.h>

#include "customlbrclient.hpp"

namespace KUKA_CONTROL
{

class KukaFRIController
{
public:
    KukaFRIController(control_mode target_mode = JOINT_POSITION, uint16_t port=30200, const std::string hostname = "");
    ~KukaFRIController();
    void start();
    void stop();

    bool setTargetJointPosition(jarray target_joint_position);
    bool setTargetJointTorque(jarray target_joint_torque);
        // bool moveJointPositionAt(jarray joint_step);
    jarray getMeasuredJointPosition();
    jarray getCommandedJointPosition();
    jarray getIpoJointPosition();
    
    jarray getMeasuredJointTorque();
    jarray getCommandedJointTorque();
    jarray getExternalJointTorque();
    //jarray getLastJointPosition();


    void lbr_application();

private:
    std::atomic<bool> running_enabled_;
    std::jthread lbr_application_thread_;

    CustomLBRClient client_;
    uint16_t application_port_;
    std::string application_hostname_;

    std::shared_ptr<jqueue> measured_joint_position_queue_;  // Meashured position [rad] queue dataflow robot -> computer
    std::shared_ptr<jqueue> commanded_joint_position_queue_; // Commanded position [rad] queue dataflow robot -> computer (to check what have been actually setten)
    std::shared_ptr<jqueue> ipo_joint_position_queue_;       // Interpolated setpoint position [rad] as basis for path superposition in queue dataflow robot -> computer

    std::shared_ptr<jqueue> measured_joint_torque_queue_;  // Meashured torque [Nm] queue dataflow robot -> computer (to read from)
    std::shared_ptr<jqueue> commanded_joint_torque_queue_; // Commanded torque [Nm] queue dataflow robot -> computer (to read from) (to check what have been actually setten)
    std::shared_ptr<jqueue> external_joint_torque_queue_;  // External torque [Nm] queue dataflow robot -> computer (to read from)

    std::shared_ptr<jqueue> commanding_joint_position_queue_; // Commanding position [rad] queue dataflow computer -> robot (to write to) (to set new position)
    std::shared_ptr<jqueue> commanding_joint_torque_queue_;   // Commanding torque [Nm] queue dataflow computer -> robot (to write to) (to set new torque)

}; // class KukaFRIController

}; // namespace KUKA_CONTROL

#endif