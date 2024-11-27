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

    std::shared_ptr<jqueue> getJointTorqueCommandingQueue();
    std::shared_ptr<jqueue> getJointPositionCommandingQueue();
    std::shared_ptr<jqueue> getActualJointPositionQueue();
    std::shared_ptr<jqueue> getActualJointTorqueQueue();

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
    std::shared_ptr<jqueue> commanded_joint_position_queue_;
    std::shared_ptr<jqueue> commanded_joint_torque_queue_;
    std::shared_ptr<jqueue> actual_joint_position_queue_;
    std::shared_ptr<jqueue> actual_joint_torque_queue_;

    jarray last_commanded_joint_torque_;
    jarray last_commanded_joint_position_;
    jarray last_actual_joint_position_;
    jarray last_actual_joint_torque_;  

    const control_mode mode_;

private:
    std::string stateToString(ESessionState state);
}; //class CustomLBRClient

}; // namespace KUKA_CONTROL

#endif