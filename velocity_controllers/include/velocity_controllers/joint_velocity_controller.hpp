 /*
  * @author Baptiste Busch
  * @date 03/04/2020
  */

#pragma once

#include <string>
#include <forward_command_controllers/forward_joint_command_controller.hpp>
#include "velocity_controllers/visibility_control.h"

namespace velocity_controllers
{
    /**
     * @class JointVelocityController
     * @brief Implemantation of a velocity controller
     */
    class JointVelocityController : public forward_command_controllers::ForwardJointCommandController
    {
    public:
        /**
         * @brief Constructor of the joint velocity controller
         */
        VELOCITY_CONTROLLERS_PUBLIC
        JointVelocityController();

        /**
         * @brief Constructor of the joint velocity controller with joint name specified
         * @param joint_name name of the joint to control
         */
        VELOCITY_CONTROLLERS_PUBLIC
        JointVelocityController(const std::string& joint_name);
    };
}  // namespace velocity_controllers
