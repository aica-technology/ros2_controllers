 /*
  * @author Baptiste Busch
  * @date 15/04/2020
  */

#pragma once

#include <string>
#include <vector>
#include <forward_command_controllers/forward_joint_group_command_controller.hpp>
#include "velocity_controllers/visibility_control.h"

namespace velocity_controllers
{
    /**
     * @class JointGroupVelocityController
     * @brief Implemantation of a velocity controller
     */
    class JointGroupVelocityController : public forward_command_controllers::ForwardJointGroupCommandController
    {
    public:
        /**
         * @brief Constructor of the joint group velocity controller
         */
        VELOCITY_CONTROLLERS_PUBLIC
        JointGroupVelocityController();

        /**
         * @brief Constructor of the joint group velocity controller with list of controlled joint specified
         * @param joint_names the list of controlled joints
         */
        VELOCITY_CONTROLLERS_PUBLIC
        JointGroupVelocityController(const std::vector<std::string>& joint_names);
    };
}  // namespace velocity_controller
