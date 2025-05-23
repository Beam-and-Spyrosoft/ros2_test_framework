// Copyright 2024 Beam Limited.
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
// @file      includes_mock.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-21
//
// @brief     Mock header for ROS 2 tools.

#pragma once

// First, disable original implementations
#include <rtest/mock_defines.hpp>

// Then include our mocks
#ifdef RTEST_SERVICE_MOCK
#include <rtest/service_base.hpp>
#include <rtest/service_mock.hpp>
#endif

#ifdef RTEST_CLIENT_MOCK
#include <rtest/client_base.hpp>
#include <rtest/service_client_mock.hpp>
#endif

#if defined(RTEST_PUBLISHER_MOCK) && defined(RTEST_SUBSCRIPTION_MOCK)
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>
#elif defined(RTEST_PUBLISHER_MOCK)
#include <rtest/publisher_mock.hpp>
#elif defined(RTEST_SUBSCRIPTION_MOCK)
#include <rtest/subscription_mock.hpp>
#endif

#ifdef RTEST_TIMER_MOCK
#include <rtest/create_timer_mock.hpp>
#endif

#ifdef RTEST_ACTION_SERVER_MOCK
#include <rtest/action_server_base.hpp>
#include <rtest/action_server_mock.hpp>
#endif

#ifdef RTEST_ACTION_CLIENT_MOCK
#include <rtest/action_client_base.hpp>
#include <rtest/action_client_mock.hpp>
#endif