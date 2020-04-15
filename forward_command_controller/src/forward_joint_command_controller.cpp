#include "forward_command_controller/forward_joint_command_controller.hpp"

namespace forward_command_controller
{
	ForwardJointCommandController::ForwardJointCommandController()
    : controller_interface::ControllerInterface()
	{}

	controller_interface::controller_interface_ret_t ForwardJointCommandController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string & controller_name)
	{
		// initialize lifecycle node
  		auto ret = ControllerInterface::init(robot_hardware, controller_name);
  		if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  		{
    		return ret;
  		}
  		// initialize joint name parameter
        this->lifecycle_node_->declare_parameter<std::string>("joint", this->joint_name_);
  		return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
	}

    controller_interface::controller_interface_ret_t ForwardJointCommandController::update()
    {
    	if (lifecycle_node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
    	return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_configure(const rclcpp_lifecycle::State&)
    {
        auto logger = this->lifecycle_node_->get_logger();
        
        // update parameters
        this->joint_name_ = this->lifecycle_node_->get_parameter("joint").as_string();

        if (auto robot_hardware = this->robot_hardware_.lock())
        {
            // register command handle
            this->registered_joint_cmd_handles_.resize(1);
            auto ret = robot_hardware->get_joint_command_handle(this->joint_name_.c_str(), &this->registered_joint_cmd_handles_[0]);
            if (ret != hardware_interface::HW_RET_OK)
            {
                RCLCPP_WARN(logger, "unable to obtain joint command handle for %s", this->joint_name_.c_str());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }
        }
        else
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // non realtime subscriber call back
        auto callback = [this, &logger](const std::shared_ptr<std_msgs::msg::Float64> msg) -> void
        {
            this->registered_joint_cmd_handles_[0]->set_cmd(msg->data);
        };

        // register subscriber
        this->joint_command_subscriber_ = this->lifecycle_node_->create_subscription<std_msgs::msg::Float64>("~/command", rclcpp::SystemDefaultsQoS(), callback);

    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_activate(const rclcpp_lifecycle::State&)
    {
    	// TODO activate controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_deactivate(const rclcpp_lifecycle::State&)
    {
    	// TODO deactivate controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_cleanup(const rclcpp_lifecycle::State&)
    {
    	// TODO clean controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_error(const rclcpp_lifecycle::State&)
    {
    	// TODO error
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointCommandController::on_shutdown(const rclcpp_lifecycle::State&)
    {
    	// TODO shutdown controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(forward_command_controller::ForwardJointCommandController, controller_interface::ControllerInterface)