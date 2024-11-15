#include "customlbrclient.hpp"

using namespace KUKA_CONTROL;

CustomLBRClient::CustomLBRClient()
  : commanded_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
    actual_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
    actual_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE))
{

}

std::shared_ptr<jqueue> CustomLBRClient::getCommandedJointPositionQueue()
{
    return commanded_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getActualJointPositionQueue()
{
    return actual_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getActualJointTorqueQueue()
{
    return actual_joint_torque_queue_;
}

void CustomLBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
    std::cout << "LBRiiwaClient state changed from [" << stateToString(oldState) << "] to [" << stateToString(newState) << "] \n";
}

void CustomLBRClient::command()
{
    std::memcpy(last_actual_joint_position_.data(), robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_actual_joint_torque_.data(), robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    actual_joint_position_queue_->push(last_actual_joint_position_);
    actual_joint_torque_queue_->push(last_actual_joint_torque_);
    last_commanded_joint_position_ = last_actual_joint_position_;
    if(commanded_joint_position_queue_->pop(last_commanded_joint_position_))
    {
        
    }
    robotCommand().setJointPosition(last_commanded_joint_position_.data());
}

std::string CustomLBRClient::stateToString(ESessionState state)
{
    switch (state)
    {
        case IDLE:
            return "Idle; No signal";
        case MONITORING_WAIT:
            return "Monitoring Wait; Poor signal";
        case MONITORING_READY:
            return "Monitoring Ready; Fair signal";
        case COMMANDING_WAIT:
            return "Commanding Wait; Good signal";
        case COMMANDING_ACTIVE:
            return "Commanding Active; Excellent (best possible) signal";
        default:
            return "Unknown state!";
    }
}