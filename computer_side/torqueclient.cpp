#include "torqueclient.hpp"

using namespace KUKA_CONTROL;

TorqueClient::TorqueClient()
  : commanded_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
    CustomLBRClient()
{

}

std::shared_ptr<jqueue> TorqueClient::getCommandedJointTorqueQueue()
{
    return commanded_joint_torque_queue_;
}

void TorqueClient::command()
{
    CustomLBRClient::command();
    last_commanded_joint_torque_ = last_actual_joint_torque_;
    if(commanded_joint_torque_queue_->pop(last_commanded_joint_torque_))
    {
        
    }
    robotCommand().setTorque(last_commanded_joint_torque_.data());
}
