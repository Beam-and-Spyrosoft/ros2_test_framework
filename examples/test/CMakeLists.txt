find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}-test
  main.cpp
  pub_sub_tests.cpp
  service_provider_tests.cpp
  service_client_tests.cpp
  debounce_tests.cpp
)

target_include_directories(${PROJECT_NAME}-test PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}-test
  ${PROJECT_NAME}::publisher_component_test_doubles
  ${PROJECT_NAME}::subscriber_component_test_doubles
  ${PROJECT_NAME}::service_provider_component_test_doubles
  ${PROJECT_NAME}::service_client_component_test_doubles
  ${PROJECT_NAME}::action_server_component_test_doubles
  ${PROJECT_NAME}::action_client_component_test_doubles
)
