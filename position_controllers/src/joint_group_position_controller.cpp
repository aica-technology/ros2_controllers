#include "position_controllers/joint_group_position_controller.hpp"

namespace position_controllers
{
	JointGroupPositionController::JointGroupPositionController()
	: forward_command_controllers::ForwardJointGroupCommandController()
	{}

	JointGroupPositionController::JointGroupPositionController(const std::vector<std::string>& joint_names)
	: forward_command_controllers::ForwardJointGroupCommandController(joint_names)
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(position_controllers::JointGroupPositionController, controller_interface::ControllerInterface)