#ifndef CUSTOM_LBR_CLIENT_HPP
#define CUSTOM_LBR_CLIENT_HPP

#include <string>
#include <iostream>
#include <array>
#include <stdint.h>
#include <cstring>
#include <memory>
#include <atomic>

#include <boost/lockfree/spsc_queue.hpp>

#include <FRI/friLBRState.h>
#include <FRI/friLBRCommand.h>
#include <FRI/friLBRClient.h>

namespace KUKA_CONTROL
{

using namespace KUKA::FRI;
using jarray = std::array<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
using jqueue = boost::lockfree::spsc_queue<jarray, boost::lockfree::fixed_sized<true>>;
constexpr uint16_t MAX_QUEUE_SIZE = 30;

enum control_mode
{
    JOINT_POSITION,
    TORQUE,
    WRENCH
};

class CustomLBRClient : public KUKA::FRI::LBRClient
{
public:
    CustomLBRClient(control_mode mode = JOINT_POSITION);

    std::shared_ptr<jqueue> getMeasuredJointPositionQueue();
    std::shared_ptr<jqueue> getCommandedJointPositionQueue();
    std::shared_ptr<jqueue> getIpoJointPositionQueue();

    std::shared_ptr<jqueue> getMeasuredJointTorqueQueue();
    std::shared_ptr<jqueue> getCommandedJointTorqueQueue();
    std::shared_ptr<jqueue> getExternalJointTorqueQueue();

    std::shared_ptr<jqueue> getJointTorqueCommandingQueue();
    std::shared_ptr<jqueue> getJointPositionCommandingQueue();

    /**
     * \brief Callback for FRI state changes.
     *
     * @param oldState
     * @param newState
     */
    virtual void onStateChange(ESessionState oldState, ESessionState newState);

    /**
     * \brief Callback for the FRI session state 'Commanding Wait'.
     */
    virtual void waitForCommand();

    /**
     * \brief Callback for the FRI state 'Commanding Active'.
     */
    virtual void command();
    
protected:
    std::shared_ptr<jqueue> measured_joint_position_queue_;  // Meashured position [rad] queue dataflow robot -> computer 
    std::shared_ptr<jqueue> commanded_joint_position_queue_; // Commanded position [rad] queue dataflow robot -> computer (to check what have been actually setten)
    std::shared_ptr<jqueue> ipo_joint_position_queue_;   // Interpolated setpoint position [rad] as basis for path superposition in queue dataflow robot -> computer

    std::shared_ptr<jqueue> measured_joint_torque_queue_;  // Meashured torque [Nm] queue dataflow robot -> computer (to read from)
    std::shared_ptr<jqueue> commanded_joint_torque_queue_; // Commanded torque [Nm] queue dataflow robot -> computer (to read from) (to check what have been actually setten)
    std::shared_ptr<jqueue> external_joint_torque_queue_;  // External torque [Nm] queue dataflow robot -> computer (to read from)

    std::shared_ptr<jqueue> commanding_joint_position_queue_; // Commanding position [rad] queue dataflow computer -> robot (to write to) (to set new position)
    std::shared_ptr<jqueue> commanding_joint_torque_queue_;  // Commanding torque [Nm] queue dataflow computer -> robot (to write to) (to set new torque)

    jarray last_measured_joint_position_;
    jarray last_commanded_joint_position_;
    jarray last_ipo_joint_position_;

    jarray last_measured_joint_torque_;
    jarray last_commanded_joint_torque_;
    jarray last_external_joint_torque_;

    jarray last_commanding_joint_position_;
    jarray last_commanding_joint_torque_;

    std::atomic<bool>
        joint_position_initialized_;
    std::atomic<bool> joint_torque_initialized_;

    const control_mode mode_;

private:
    std::string stateToString(ESessionState state);
}; //class CustomLBRClient

}; // namespace KUKA_CONTROL

#endif