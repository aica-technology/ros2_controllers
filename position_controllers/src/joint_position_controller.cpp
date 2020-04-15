#include "position_controllers/joint_position_controller.hpp"

namespace position_controllers
{
	JointPositionController::JointPositionController()
	: forward_command_controllers::ForwardJointCommandController()
	{}

	JointPositionController::JointPositionController(const std::string& joint_name)
	: forward_command_controllers::ForwardJointCommandController(joint_name)
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(position_controllers::JointPositionController, controller_interface::ControllerInterface)