#include "forward_command_controllers/forward_joint_group_command_controller.hpp"

namespace forward_command_controllers
{
	ForwardJointGroupCommandController::ForwardJointGroupCommandController()
    : controller_interface::ControllerInterface()
	{}

	controller_interface::controller_interface_ret_t ForwardJointGroupCommandController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string & controller_name)
	{
		// initialize lifecycle node
  		auto ret = ControllerInterface::init(robot_hardware, controller_name);
  		if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  		{
    		return ret;
  		}
  		// initialize joint name parameter
        this->lifecycle_node_->declare_parameter<std::vector<std::string>>("joints", this->joint_names_);
  		return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
	}

    controller_interface::controller_interface_ret_t ForwardJointGroupCommandController::update()
    {
    	if (lifecycle_node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
    	return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_configure(const rclcpp_lifecycle::State&)
    {
        auto logger = this->lifecycle_node_->get_logger();
        
        // update parameters
        this->joint_names_ = this->lifecycle_node_->get_parameter("joints").as_string_array();

        if (auto robot_hardware = this->robot_hardware_.lock())
        {
            // register command handle
            this->registered_joint_cmd_handles_.resize(this->joint_names_.size());
            for (size_t index = 0; index < this->joint_names_.size(); ++index)
            {
                auto ret = robot_hardware->get_joint_command_handle(this->joint_names_[index].c_str(), &this->registered_joint_cmd_handles_[index]);
                if (ret != hardware_interface::HW_RET_OK)
                {
                    RCLCPP_WARN(logger, "unable to obtain joint command handle for %s", this->joint_names_[index].c_str());
                    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
                }
            }
        }
        else
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        if (this->registered_joint_cmd_handles_.empty()) return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

        // non realtime subscriber call back
        auto callback = [this, &logger](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) -> void
        {
            if (this->registered_joint_cmd_handles_.size() != msg->data.size())
            {
                RCLCPP_ERROR(logger,
                             "number of joints in joint trajectory msg (%d) "
                             "does not match number of joint command handles (%d)\n",
                             msg->data.size(), this->registered_joint_cmd_handles_.size()
                            );
            }
            for (size_t index = 0; index < msg->data.size(); ++index)
            {
                this->registered_joint_cmd_handles_[index]->set_cmd(msg->data[index]);
            }
        };

        // register subscriber
        this->joint_command_subscriber_ = this->lifecycle_node_->create_subscription<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::SystemDefaultsQoS(), callback);

    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_activate(const rclcpp_lifecycle::State&)
    {
    	// TODO activate controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_deactivate(const rclcpp_lifecycle::State&)
    {
    	// TODO deactivate controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_cleanup(const rclcpp_lifecycle::State&)
    {
    	// TODO clean controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_error(const rclcpp_lifecycle::State&)
    {
    	// TODO error
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ForwardJointGroupCommandController::on_shutdown(const rclcpp_lifecycle::State&)
    {
    	// TODO shutdown controller
    	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}
