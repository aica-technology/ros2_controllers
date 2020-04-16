 /*
  * @author Baptiste Busch
  * @date 15/04/2020
  */

#pragma once

#include <string>
#include <vector>
#include <forward_command_controllers/forward_joint_group_command_controller.hpp>
#include "position_controllers/visibility_control.h"

namespace position_controllers
{
    /**
     * @class JointGroupPositionController
     * @brief Implemantation of a position controller
     */
    class JointGroupPositionController : public forward_command_controllers::ForwardJointGroupCommandController
    {
    public:
        /**
         * @brief Constructor of the joint group position controller
         */
        POSITION_CONTROLLERS_PUBLIC
        JointGroupPositionController();

        /**
         * @brief Constructor of the joint group position controller with list of controlled joint specified
         * @param joint_names the list of controlled joints
         */
        POSITION_CONTROLLERS_PUBLIC
        JointGroupPositionController(const std::vector<std::string>& joint_names);
    };
}  // namespace position_controllers
