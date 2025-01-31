#ifndef TORQUE_CLIENT_HPP
#define TORQUE_CLIENT_HPP

#include "customlbrclient.hpp"

namespace KUKA_CONTROL
{

class TorqueClient : public CustomLBRClient
{
public:
    TorqueClient();

    std::shared_ptr<jqueue> getCommandedJointTorqueQueue();

    /**
     * \brief Callback for the FRI state 'Commanding Active'.
     */
    virtual void command();
    
protected:
    std::shared_ptr<jqueue> commanded_joint_torque_queue_;

    jarray last_commanded_joint_torque_;
}; //class TorqueClient

}; // namespace KUKA_CONTROL

#endif