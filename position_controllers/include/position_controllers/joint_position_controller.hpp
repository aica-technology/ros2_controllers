 /*
  * @author Baptiste Busch
  * @date 03/04/2020
  */

#pragma once

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
         * @brief Constructor of the position controller
         */
        POSITION_CONTROLLERS_PUBLIC
        JointPositionController();
    };
}  // namespace position_controller
