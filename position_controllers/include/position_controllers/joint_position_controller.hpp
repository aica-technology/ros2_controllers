 /*
  * @author Baptiste Busch
  * @date 03/04/2020
  */

#pragma once

#include <string>
#include <forward_command_controllers/forward_joint_command_controller.hpp>
#include "position_controllers/visibility_control.h"

namespace position_controllers
{
    /**
     * @class JointPositionController
     * @brief Implemantation of a position controller
     */
    class JointPositionController : public forward_command_controllers::ForwardJointCommandController
    {
    public:
        /**
         * @brief Constructor of the joint position controller
         */
        POSITION_CONTROLLERS_PUBLIC
        JointPositionController();

        /**
         * @brief Constructor of the joint position controller with joint name specified
         * @param joint_name name of the joint to control
         */
        POSITION_CONTROLLERS_PUBLIC
        JointPositionController(const std::string& joint_name);
    };
}  // namespace position_controllers
