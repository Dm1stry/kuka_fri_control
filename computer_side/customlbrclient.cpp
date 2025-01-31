#include "customlbrclient.hpp"

// #include <stdio.h>

using namespace KUKA_CONTROL;

CustomLBRClient::CustomLBRClient(control_mode mode)
    : commanded_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
      commanded_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      actual_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
      actual_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      joint_position_initialized_(false),
      mode_(mode)
{
}

std::shared_ptr<jqueue> CustomLBRClient::getJointTorqueCommandingQueue()
{
    return commanded_joint_torque_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getJointPositionCommandingQueue()
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

void CustomLBRClient::waitForCommand()
{
    //LBRClient::waitForCommand();
    robotCommand().setJointPosition(last_commanded_joint_position_.data());
    if (mode_ == TORQUE && robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
    {
        robotCommand().setTorque(last_commanded_joint_torque_.data());
    }
}

void CustomLBRClient::command()
{
    // printf("%f", robotState().getSampleTime());
    std::memcpy(last_actual_joint_position_.data(), robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_actual_joint_torque_.data(), robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    actual_joint_position_queue_->push(last_actual_joint_position_);
    actual_joint_torque_queue_->push(last_actual_joint_torque_);
    switch(mode_)
    {
    case TORQUE:
        LBRClient::command();
        if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
        {
            if (!joint_torque_initialized_.load())
            {
                last_commanded_joint_torque_ = last_actual_joint_torque_;
                joint_torque_initialized_.store(true);
            }
            if (commanded_joint_torque_queue_->pop(last_commanded_joint_torque_))
            {
            }
            robotCommand().setTorque(last_commanded_joint_torque_.data());
        }
    case JOINT_POSITION:
        if (!joint_position_initialized_.load())
        {
            last_commanded_joint_position_ = last_actual_joint_position_;
            joint_position_initialized_.store(true);
        }
        if (commanded_joint_position_queue_->pop(last_commanded_joint_position_))
        {
        }
        robotCommand().setJointPosition(last_commanded_joint_position_.data());
        break;
    }
    
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