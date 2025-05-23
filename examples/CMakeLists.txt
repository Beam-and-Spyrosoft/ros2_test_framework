cmake_minimum_required(VERSION 3.10.2)
project(rtest_examples 
  LANGUAGES CXX
  VERSION 0.1.0
)

# Build tests by default
option(BUILD_TESTING "Build tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)

# Publisher component
add_library(publisher_component SHARED src/publisher.cpp)
target_compile_definitions(publisher_component PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(publisher_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(publisher_component "test_composition::Publisher")

# Subscriber component
add_library(subscriber_component SHARED src/subscriber.cpp)
target_compile_definitions(subscriber_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(subscriber_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(subscriber_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(subscriber_component "test_composition::Subscriber")

# Service provider component
add_library(service_provider_component SHARED src/service_provider.cpp)
target_compile_definitions(service_provider_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(service_provider_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(service_provider_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)
rclcpp_components_register_nodes(service_provider_component "test_composition::ServiceProvider")

# Service client component
add_library(service_client_component SHARED src/service_client.cpp)
target_compile_definitions(service_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(service_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(service_client_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)
rclcpp_components_register_nodes(service_client_component "test_composition::ServiceClient")


# Action server component
add_library(action_server_component SHARED src/action_server.cpp)
target_compile_definitions(action_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(action_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(action_server_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)
rclcpp_components_register_nodes(action_server_component "test_composition::ActionServer")

# Action client component
add_library(action_client_component SHARED src/action_client.cc)
target_compile_definitions(action_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(action_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(action_client_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)
rclcpp_components_register_nodes(action_client_component "test_composition::ActionClient")


# Manual/static Node composition
add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition
  publisher_component
  subscriber_component
  service_provider_component
  service_client_component
  action_server_component
  action_client_component
)
ament_target_dependencies(manual_composition
  rclcpp
)

install(TARGETS
  publisher_component
  subscriber_component
  service_provider_component
  service_client_component
  action_server_component
  action_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(rtest REQUIRED)
  # Option to enable clang-tidy tests (default: disabled)
  option(ENABLE_CLANG_TIDY "Enable clang-tidy tests (can be time-consuming)" OFF)
  if(ENABLE_CLANG_TIDY)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)
    set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-tidy)
    ament_lint_auto_find_test_dependencies()
  else()
    find_package(ament_cmake_copyright REQUIRED)
    ament_copyright()

    test_tools_add_doubles(publisher_component
      AMENT_DEPENDENCIES
        rclcpp
        rclcpp_components
        std_msgs
    )
    test_tools_add_doubles(subscriber_component
      AMENT_DEPENDENCIES
        rclcpp
        rclcpp_components
        std_msgs
    )
    test_tools_add_doubles(service_provider_component
      AMENT_DEPENDENCIES
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
    )
    test_tools_add_doubles(service_client_component
      AMENT_DEPENDENCIES
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
    )
    test_tools_add_doubles(action_server_component
    AMENT_DEPENDENCIES
      rclcpp
      rclcpp_components
      std_msgs
      std_srvs
    )
    test_tools_add_doubles(action_client_component
      AMENT_DEPENDENCIES
        rclcpp
        rclcpp_components
        std_msgs
        std_srvs
    )
    add_subdirectory(test)
  endif()
endif()

ament_package()
