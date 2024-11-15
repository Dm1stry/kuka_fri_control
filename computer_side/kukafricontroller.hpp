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
    KukaFRIController(uint16_t port=30200, const std::string hostname = "");
    ~KukaFRIController();
    void start();
    void stop();

    bool setJointPosition(jarray joint_position);
    bool moveJointPositionAt(jarray joint_step);
    //jarray getLastJointPosition();


    void lbr_application();

private:
    std::atomic<bool> running_enabled_;
    std::jthread lbr_application_thread_;

    CustomLBRClient client_;
    uint16_t application_port_;
    std::string application_hostname_;

    std::shared_ptr<jqueue> commanded_joint_position_queue_;
    std::shared_ptr<jqueue> actual_joint_position_queue_;
    std::shared_ptr<jqueue> actual_joint_torque_queue_;

}; // class KukaFRIController

}; // namespace KUKA_CONTROL

#endif