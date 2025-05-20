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
// @file      main.cpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-12-04

#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  /// Run all tests
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
