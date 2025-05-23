#pragma once

#include <memory>
#include <functional>
#include "rclcpp/waitable.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action {

enum class GoalResponse : int8_t {
  REJECT = 1,
  ACCEPT_AND_EXECUTE = 2,
  ACCEPT_AND_DEFER = 3,
};

enum class CancelResponse : int8_t {
  REJECT = 1,
  ACCEPT = 2,
};

class ServerBase : public rclcpp::Waitable {
public:
  enum class EntityType : std::size_t {
    GoalService,
    ResultService,
    CancelService,
    Expired,
  };

  // Constructor - interface only
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_server_options_t & options) = default;

  virtual ~ServerBase() = default;

  // Pure virtual interface methods for goal handling
  virtual std::pair<GoalResponse, std::shared_ptr<void>> call_handle_goal_callback(GoalUUID & uuid, std::shared_ptr<void> request) = 0;
  virtual CancelResponse call_handle_cancel_callback(const GoalUUID & uuid) = 0;
  virtual void call_goal_accepted_callback(std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle, GoalUUID uuid, std::shared_ptr<void> goal_request_message) = 0;

  // Message creation interface
  virtual GoalUUID get_goal_id_from_goal_request(void * message) = 0;
  virtual std::shared_ptr<void> create_goal_request() = 0;
  virtual GoalUUID get_goal_id_from_result_request(void * message) = 0;
  virtual std::shared_ptr<void> create_result_request() = 0;
  virtual std::shared_ptr<void> create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) = 0;

  // Publishing interface
  virtual void publish_status() = 0;
  virtual void notify_goal_terminal_state() = 0;
  virtual void publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg) = 0;
  virtual void publish_feedback(std::shared_ptr<void> feedback_msg) = 0;

  // Waitable interface - default implementations
  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override {}
  bool is_ready(const rcl_wait_set_t & wait_set) override { return false; }
  std::shared_ptr<void> take_data() override { return nullptr; }
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override { return nullptr; }
  void execute(const std::shared_ptr<void> & data) override {}

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override {}
  void clear_on_ready_callback() override {}
};

} // namespace rclcpp_action