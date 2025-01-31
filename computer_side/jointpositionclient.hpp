#ifndef JOINT_POSITION_CLIENT_HPP
#define JOINT_POSITION_CLIENT_HPP

#include "customlbrclient.hpp"

namespace KUKA_CONTROL
{

class JointPositionClient : public CustomLBRClient
{
public:
    JointPositionClient();

    std::shared_ptr<jqueue> getCommandedJointPositionQueue();

    /**
     * \brief Callback for the FRI state 'Commanding Active'.
     */
    virtual void command();
    
protected:
    std::shared_ptr<jqueue> commanded_joint_position_queue_;

    jarray last_commanded_joint_position_;
}; //class JointPositionClient

}; // namespace KUKA_CONTROL

#endif