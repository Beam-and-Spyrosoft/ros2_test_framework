# Copyright 2024 Beam Limited.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# @file      test_tools_add_doubles.cmake
# @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
# @date      2024-11-25
#
# @details This macro creates a test doubles library with mocked ROS 2 rclcpp layer.
#          It is used to create a library that can be used in integration tests
#          to replace the original implementation of a target with a test double.
#          The test double library is created with the same sources as the original
#          implementation, but with additional test doubles for the ROS 2 rclcpp layer.




# @brief Create and install a test doubles library for a given target.
#
# This macro creates a test doubles library with mocked ROS 2 rclcpp layer.
#
# @param target Name of the target to create test doubles for. Will create library
#               named <target>_test_doubles and alias target <project_name>::<target>_test_doubles
# @param SOURCES List of source files to be compiled into the test doubles library
# @param AMENT_DEPENDENCIES (optional) Additional dependencies to be linked with the test library
#
# @example
#   project(robot)
#   find_package(my_robot_test_doubles REQUIRED)
#
#   # Add original implementation target
#   add_library(my_robot
#     src/sensor.cpp
#     src/actuator.cpp)
#
#   set(AMENT_TARGET_DEPS Eigen3 rclcpp std_msgs)
#   ament_target_dependencies(my_robot ${AMENT_TARGET_DEPS})
#
#   # Create test doubles target for the original implementation
#   test_tools_add_doubles(my_robot
#     AMENT_DEPENDENCIES ${AMENT_TARGET_DEPS})
#
#   ---
#
#   # In another package that uses the test doubles
#   
#   add_executable(integration_test test/main.cpp)
#   target_link_libraries(integration_test
#     robot::my_robot_test_doubles
#   )
#
function(test_tools_add_doubles target)
  include(CMakePackageConfigHelpers)

  cmake_parse_arguments(arg "INTERFACE" "" "SOURCES;AMENT_DEPENDENCIES" ${ARGN})

  set(LIB_DOUBLES_NAME ${target}_test_doubles)

  if(arg_INTERFACE)
    add_library(${LIB_DOUBLES_NAME} INTERFACE)
    add_library(${PROJECT_NAME}::${LIB_DOUBLES_NAME} ALIAS ${LIB_DOUBLES_NAME})

    target_include_directories(${LIB_DOUBLES_NAME} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )

    target_link_libraries(${LIB_DOUBLES_NAME} INTERFACE
      rtest::publisher_mock
      rtest::subscription_mock
      rtest::service_mock
      rtest::service_client_mock
      rtest::timer_mock
    )

    ament_target_dependencies(${LIB_DOUBLES_NAME} INTERFACE
      ${arg_AMENT_DEPENDENCIES}
    )
  else()
    get_target_property(TARGET_SOURCES ${target} SOURCES)

    add_library(${LIB_DOUBLES_NAME} ${TARGET_SOURCES} ${arg_SOURCES})
    add_library(${PROJECT_NAME}::${LIB_DOUBLES_NAME} ALIAS ${LIB_DOUBLES_NAME})
    set_property(TARGET ${LIB_DOUBLES_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

    target_include_directories(${LIB_DOUBLES_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    )

    target_link_libraries(${LIB_DOUBLES_NAME}
      rtest::publisher_mock
      rtest::subscription_mock
      rtest::service_mock
      rtest::service_client_mock
      rtest::timer_mock
      rtest::action_server_mock
      rtest::action_client_mock
      rtest::rtest_common
    )

    ament_target_dependencies(${LIB_DOUBLES_NAME}
      ${arg_AMENT_DEPENDENCIES}
    )
  endif()
  
  
  # set(RTEST_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/../rtest/share/rtest/cmake/")
  set(RTEST_CMAKE_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")


  configure_package_config_file(
    ${RTEST_CMAKE_DIR}/test-doublesConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_DOUBLES_NAME}Config.cmake
    INSTALL_DESTINATION share/${LIB_DOUBLES_NAME}/cmake
  )

  install(TARGETS ${LIB_DOUBLES_NAME}
    EXPORT ${LIB_DOUBLES_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  install(EXPORT ${LIB_DOUBLES_NAME}_targets
    FILE ${LIB_DOUBLES_NAME}_targetsExport.cmake
    NAMESPACE ${target}::
    DESTINATION share/${LIB_DOUBLES_NAME}/cmake/
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${LIB_DOUBLES_NAME}Config.cmake
    DESTINATION share/${LIB_DOUBLES_NAME}/cmake/
  )
endfunction()