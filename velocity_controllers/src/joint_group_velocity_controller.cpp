#include "velocity_controllers/joint_group_velocity_controller.hpp"

namespace velocity_controllers
{
	JointGroupVelocityController::JointGroupVelocityController()
	: forward_command_controllers::ForwardJointGroupCommandController()
	{}

	JointGroupVelocityController::JointGroupVelocityController(const std::vector<std::string>& joint_names)
	: forward_command_controllers::ForwardJointGroupCommandController(joint_names)
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(velocity_controllers::JointGroupVelocityController, controller_interface::ControllerInterface)