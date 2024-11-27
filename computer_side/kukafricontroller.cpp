#include "kukafricontroller.hpp"

using namespace KUKA_CONTROL;

KukaFRIController::KukaFRIController(control_mode target_mode, uint16_t port, const std::string hostname)
	: running_enabled_(false),
	  client_(target_mode),
	  application_port_(port),
	  application_hostname_(hostname),
	  commanded_joint_position_queue_(client_.getJointPositionCommandingQueue()),
	  commanded_joint_torque_queue_(client_.getJointTorqueCommandingQueue()),
	  actual_joint_position_queue_(client_.getActualJointPositionQueue()),
	  actual_joint_torque_queue_(client_.getActualJointTorqueQueue())
{

}

KukaFRIController::~KukaFRIController()
{
	running_enabled_.store(false);
}

void KukaFRIController::start()
{
	running_enabled_.store(true);
	lbr_application_thread_ = std::jthread(  // Create new thread for lbr_application execution
			std::bind(&KukaFRIController::lbr_application, this)  // Make real to call class member function just like a simple function
		);
}

void KukaFRIController::stop()
{
	running_enabled_.store(false);
	lbr_application_thread_.join();
}

bool KukaFRIController::setTargetJointPosition(jarray target_joint_position)
{
	return commanded_joint_position_queue_->push(target_joint_position);
}

bool KukaFRIController::setTargetJointTorque(jarray target_joint_torque)
{
	return commanded_joint_torque_queue_->push(target_joint_torque);
}
// bool KukaFRIController::moveJointPositionAt(jarray joint_step)
// {
// 	jarray current_position;
// 	if(actual_joint_position_queue_->pop(current_position))
// 	{
// 		for(int i = 0; i < current_position.size(); ++i)
// 		{
// 			current_position[i] += joint_step[i];
// 		}
// 		return commanded_queue_->push(current_position);
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

jarray KukaFRIController::getJointPosition()
{
	jarray joint_position;
	while(!actual_joint_position_queue_->pop(joint_position)) {}
	return joint_position;
}

jarray KukaFRIController::getTorque()
{
	jarray torque;
	while(!actual_joint_torque_queue_->pop(torque)) {}
	return torque;

}

void KukaFRIController::lbr_application()
{
	UdpConnection connection;
	ClientApplication app(connection, client_);
	if (application_hostname_.empty())
	{
		app.connect(application_port_, NULL);
	}
	else
	{
		app.connect(application_port_, application_hostname_.c_str());
	}
	
	bool success = true;
	while(success && running_enabled_.load())
	{	
		success = app.step();
	}
}