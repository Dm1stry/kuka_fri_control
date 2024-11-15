#include "kukafricontroller.hpp"

using namespace KUKA_CONTROL;

KukaFRIController::KukaFRIController(uint16_t port, const std::string hostname)
  : running_enabled_(false),
  	client_(),
	application_port_(port),
	application_hostname_(hostname),
	commanded_joint_position_queue_(client_.getCommandedJointPositionQueue()),
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

bool KukaFRIController::setJointPosition(jarray joint_position)
{
	return actual_joint_position_queue_->push(joint_position);
}

bool KukaFRIController::moveJointPositionAt(jarray joint_step)
{
	jarray current_position;
	if(actual_joint_position_queue_->pop(current_position))
	{
		for(int i = 0; i < current_position.size(); ++i)
		{
			current_position[i] += joint_step[i];
		}
		return commanded_joint_position_queue_->push(current_position);
	}
	else
	{
		return false;
	}
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