#include "position_controllers/joint_group_position_controller.hpp"

namespace position_controllers
{
	JointGroupPositionController::JointGroupPositionController()
	: forward_command_controller::ForwardJointGroupCommandController()
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(position_controllers::JointGroupPositionController, controller_interface::ControllerInterface)