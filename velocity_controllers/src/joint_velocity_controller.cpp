#include "velocity_controllers/joint_velocity_controller.hpp"

namespace velocity_controllers
{
	JointVelocityController::JointVelocityController()
	: forward_command_controllers::ForwardJointCommandController()
	{}

	JointVelocityController::JointVelocityController(const std::string& joint_name)
	: forward_command_controllers::ForwardJointCommandController(joint_name)
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(velocity_controllers::JointVelocityController, controller_interface::ControllerInterface)