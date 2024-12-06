#include "customlbrclient.hpp"

// #include <stdio.h>

using namespace KUKA_CONTROL;

CustomLBRClient::CustomLBRClient(control_mode mode)
    : measured_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
      commanded_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
      ipo_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),
      measured_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      commanded_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      external_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      commanding_joint_torque_queue_(new jqueue(MAX_QUEUE_SIZE)),
      commanding_joint_position_queue_(new jqueue(MAX_QUEUE_SIZE)),

      joint_position_initialized_(false),
      mode_(mode)
{
}

std::shared_ptr<jqueue> CustomLBRClient::getJointTorqueCommandingQueue()
{
    return commanding_joint_torque_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getJointPositionCommandingQueue()
{
    return commanding_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getMeasuredJointPositionQueue()
{
    return measured_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getCommandedJointPositionQueue()
{
    return commanded_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getIpoJointPositionQueue()
{
    return ipo_joint_position_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getMeasuredJointTorqueQueue()
{
    return measured_joint_torque_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getCommandedJointTorqueQueue()
{
    return commanded_joint_torque_queue_;
}

std::shared_ptr<jqueue> CustomLBRClient::getExternalJointTorqueQueue()
{
    return external_joint_torque_queue_;
}

void CustomLBRClient::onStateChange(ESessionState oldState, ESessionState newState)
{
    std::cout << "LBRiiwaClient state changed from [" << stateToString(oldState) << "] to [" << stateToString(newState) << "] \n";
}

void CustomLBRClient::waitForCommand()
{
    //LBRClient::waitForCommand();
    robotCommand().setJointPosition(last_commanding_joint_position_.data());
    if (mode_ == TORQUE && robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
    {
        robotCommand().setTorque(last_commanding_joint_torque_.data());
    }
}

void CustomLBRClient::command()
{
    // printf("%f", robotState().getSampleTime());
    std::memcpy(last_measured_joint_position_.data(), robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_commanded_joint_position_.data(), robotState().getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_ipo_joint_position_.data(), robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

    std::memcpy(last_measured_joint_torque_.data(), robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_commanded_joint_torque_.data(), robotState().getCommandedTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    std::memcpy(last_external_joint_torque_.data(), robotState().getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

    measured_joint_position_queue_->push(last_measured_joint_position_);
    commanding_joint_position_queue_->push(last_commanded_joint_position_);
    ipo_joint_position_queue_->push(last_ipo_joint_position_);

    measured_joint_torque_queue_->push(last_measured_joint_torque_);
    commanded_joint_torque_queue_->push(last_commanded_joint_torque_);
    external_joint_torque_queue_->push(last_external_joint_torque_);

    switch (mode_)
    {
    case TORQUE:
        LBRClient::command();
        if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE)
        {
            if (!joint_torque_initialized_.load())
            {
                last_commanding_joint_torque_ = last_measured_joint_torque_;
                joint_torque_initialized_.store(true);
            }
            if (commanding_joint_torque_queue_->pop(last_commanding_joint_torque_))
            {
            }
            robotCommand().setTorque(last_commanding_joint_torque_.data());
        }
    case JOINT_POSITION:
        if (!joint_position_initialized_.load())
        {
            last_commanded_joint_position_ = last_measured_joint_position_;
            joint_position_initialized_.store(true);
        }
        if (commanding_joint_position_queue_->pop(last_commanding_joint_position_))
        {
        }
        robotCommand().setJointPosition(last_commanding_joint_position_.data());
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