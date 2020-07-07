 /*
  * @author Baptiste Busch
  * @date 03/04/2020
  */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/operation_mode_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "forward_command_controllers/visibility_control.h"

namespace forward_command_controllers
{
    /**
     * @class ForwardJointCommandController
     * @brief Implemantation of a forward_command controller
     */
    class ForwardJointCommandController : public controller_interface::ControllerInterface
    {
    private:
        std::vector<hardware_interface::JointCommandHandle *> registered_joint_cmd_handles_; ///< handler of the command to send to the robot
        std::string joint_name_; ///< name of the controlled joint
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_command_subscriber_ = nullptr; ///< pointer to the desired command value subscription

    public:
        /**
         * @brief Constructor of the forward joint command controller
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        ForwardJointCommandController();

        /**
         * @brief Constructor of the forward joint command controller with joint name specified
         * @param joint_name name of the joint to control
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        ForwardJointCommandController(const std::string& joint_name);

        /**
         * @brief Init function called to initialize the controller
         * @param robot_hardware pointer to the hardware interface
         * @param controller_name name of the controller
         * @return controller_interface::return_type success of failure return code
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        controller_interface::return_type init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string & controller_name) override;

        /**
         * @brief Update function called to read the robot current state and generate the command
         * @return controller_interface::return_type success of failure return code
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        controller_interface::return_type update() override;

        /**
         * @brief Transition callback for state configuring
         *
         * on_configure callback is being called when the lifecycle node
         * enters the "configuring" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "unconfigured".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        
        /**
         * @brief Transition callback for state activating
         *
         * on_activate callback is being called when the lifecycle node
         * enters the "activating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "active" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "active"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief Transition callback for state deactivating
         *
         * on_deactivate callback is being called when the lifecycle node
         * enters the "deactivating" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "inactive" state or stays
         * in "active".
         * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
         * TRANSITION_CALLBACK_FAILURE transitions to "active"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief Transition callback for state cleaningup
         *
         * on_cleanup callback is being called when the lifecycle node
         * enters the "cleaningup" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "unconfigured" state or stays
         * in "inactive".
         * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
         * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief Transition callback for state throwing an error
         *
         * on_shutdown callback is being called when the lifecycle node
         * enters the "error" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "finalized" state or stays
         * in its current state.
         * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
         * TRANSITION_CALLBACK_FAILURE transitions to current state
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief Transition callback for state shutting down
         *
         * on_shutdown callback is being called when the lifecycle node
         * enters the "shuttingdown" state.
         * Depending on the return value of this function, the state machine
         * either invokes a transition to the "finalized" state or stays
         * in its current state.
         * TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
         * TRANSITION_CALLBACK_FAILURE transitions to current state
         * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
         */
        FORWARD_COMMAND_CONTROLLERS_PUBLIC
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    };
}  // namespace forward_command_controllers
