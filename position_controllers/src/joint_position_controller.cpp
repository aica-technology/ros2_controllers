#include "position_controllers/joint_position_controller.hpp"

namespace position_controllers
{
	JointPositionController::JointPositionController()
	: forward_command_controller::ForwardJointCommandController()
	{}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(position_controllers::JointPositionController, controller_interface::ControllerInterface)