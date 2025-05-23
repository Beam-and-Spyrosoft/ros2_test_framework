// Copyright 2025 Spyrosoft Limited.
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
//
// @file      mock_defines.hpp
// @author    Mariusz Szczepanik (mua@spyro-soft.com)
// @date      2025-05-28

#pragma once

// Disable original ROS2 implementations

#ifdef RTEST_PUBLISHER_MOCK
#define RCLCPP__PUBLISHER_HPP_
#endif

#ifdef RTEST_SUBSCRIPTION_MOCK
#define RCLCPP__SUBSCRIPTION_HPP_
#endif

#ifdef RTEST_SERVICE_MOCK
#define RCLCPP__SERVICE_HPP_
#endif

#ifdef RTEST_CLIENT_MOCK
#define RCLCPP__CLIENT_HPP_
#endif

#ifdef RTEST_ACTION_SERVER_MOCK
#define RCLCPP_ACTION__SERVER_HPP_
#endif

#ifdef RTEST_ACTION_CLIENT_MOCK
#define RCLCPP_ACTION__CLIENT_HPP_
#endif

#ifdef RTEST_TIMER_MOCK
#define RCLCPP__CREATE_TIMER_HPP_
#endif