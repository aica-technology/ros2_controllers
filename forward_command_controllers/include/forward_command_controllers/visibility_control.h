// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef FORWARD_COMMAND_CONTROLLERS__VISIBILITY_CONTROL_H_
#define FORWARD_COMMAND_CONTROLLERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FORWARD_COMMAND_CONTROLLERS_EXPORT __attribute__ ((dllexport))
    #define FORWARD_COMMAND_CONTROLLERS_IMPORT __attribute__ ((dllimport))
  #else
    #define FORWARD_COMMAND_CONTROLLERS_EXPORT __declspec(dllexport)
    #define FORWARD_COMMAND_CONTROLLERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FORWARD_COMMAND_CONTROLLER_BUILDING_DLL
    #define FORWARD_COMMAND_CONTROLLERS_PUBLIC FORWARD_COMMAND_CONTROLLERS_EXPORT
  #else
    #define FORWARD_COMMAND_CONTROLLERS_PUBLIC FORWARD_COMMAND_CONTROLLERS_IMPORT
  #endif
  #define FORWARD_COMMAND_CONTROLLERS_PUBLIC_TYPE FORWARD_COMMAND_CONTROLLERS_PUBLIC
  #define FORWARD_COMMAND_CONTROLLERS_LOCAL
#else
  #define FORWARD_COMMAND_CONTROLLERS_EXPORT __attribute__ ((visibility("default")))
  #define FORWARD_COMMAND_CONTROLLERS_IMPORT
  #if __GNUC__ >= 4
    #define FORWARD_COMMAND_CONTROLLERS_PUBLIC __attribute__ ((visibility("default")))
    #define FORWARD_COMMAND_CONTROLLERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FORWARD_COMMAND_CONTROLLERS_PUBLIC
    #define FORWARD_COMMAND_CONTROLLERS_LOCAL
  #endif
  #define FORWARD_COMMAND_CONTROLLERS_PUBLIC_TYPE
#endif

#endif  // FORWARD_COMMAND_CONTROLLERS__VISIBILITY_CONTROL_H_
