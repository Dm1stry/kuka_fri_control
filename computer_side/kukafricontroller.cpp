#include "kukafricontroller.hpp"

using namespace KUKA_CONTROL;

KukaFRIController::KukaFRIController(control_mode target_mode, uint16_t port, const std::string hostname)
	: running_enabled_(false),
	  client_(target_mode),
	  application_port_(port),
	  application_hostname_(hostname),
	  measured_joint_position_queue_(client_.getMeasuredJointPositionQueue()),
	  commanded_joint_position_queue_(client_.getCommandedJointPositionQueue()),
	  ipo_joint_position_queue_(client_.getIpoJointPositionQueue()),
	  measured_joint_torque_queue_(client_.getMeasuredJointTorqueQueue()),
	  commanded_joint_torque_queue_(client_.getCommandedJointTorqueQueue()),
	  external_joint_torque_queue_(client_.getExternalJointTorqueQueue()),
	  commanding_joint_position_queue_(client_.getJointPositionCommandingQueue()),
	  commanding_joint_torque_queue_(client_.getJointTorqueCommandingQueue())
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
	return commanding_joint_position_queue_->push(target_joint_position);
}

bool KukaFRIController::setTargetJointTorque(jarray target_joint_torque)
{
	return commanding_joint_torque_queue_->push(target_joint_torque);
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

jarray KukaFRIController::getMeasuredJointPosition()
{
	jarray joint_position;
	while(!measured_joint_position_queue_->pop(joint_position)) {}
	return joint_position;
}

jarray KukaFRIController::getCommandedJointPosition()
{
	jarray joint_position;
	while (!commanded_joint_position_queue_->pop(joint_position)) {}
	return joint_position;
}

jarray KukaFRIController::getIpoJointPosition()
{
	jarray joint_position;
	while(!ipo_joint_position_queue_->pop(joint_position)) {}
	return joint_position;
}

jarray KukaFRIController::getMeasuredJointTorque()
{
	jarray torque;
	while (!measured_joint_torque_queue_->pop(torque)) {}
	return torque;
}

jarray KukaFRIController::getCommandedJointTorque()
{
	jarray torque;
	while (!commanded_joint_torque_queue_->pop(torque)) {}
	return torque;
}

jarray KukaFRIController::getExternalJointTorque()
{
	jarray torque;
	while (!external_joint_torque_queue_->pop(torque)) {}
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