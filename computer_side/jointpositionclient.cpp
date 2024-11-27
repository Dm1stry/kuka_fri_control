#include "jointpositionclient.hpp"

using namespace KUKA_CONTROL;

JointPositionClient::JointPositionClient()
  : commanded_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
    CustomLBRClient()
{

}

std::shared_ptr<jqueue> JointPositionClient::getCommandedJointPositionQueue()
{
    return commanded_joint_position_queue_;
}

void JointPositionClient::command()
{
    CustomLBRClient::command();
    last_commanded_joint_position_ = last_actual_joint_position_;
    if(commanded_joint_position_queue_->pop(last_commanded_joint_position_))
    {
        
    }
    robotCommand().setJointPosition(last_commanded_joint_position_.data());
}
