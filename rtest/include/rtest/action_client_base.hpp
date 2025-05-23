#pragma once

#include <memory>
#include <functional>
#include <chrono>
#include "rclcpp/waitable.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action {

class ClientBase : public rclcpp::Waitable {
public:
  using ResponseCallback = std::function<void(std::shared_ptr<void>)>;

  enum class EntityType : std::size_t {
    GoalClient,
    ResultClient,
    CancelClient,
    FeedbackSubscription,
    StatusSubscription,
  };

  // Constructor - minimal, just for interface
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & client_options) = default;

  virtual ~ClientBase() = default;

  // Pure virtual interface methods
  virtual bool action_server_is_ready() const = 0;
  virtual bool wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout) = 0;
  virtual rclcpp::Logger get_logger() = 0;
  virtual GoalUUID generate_goal_id() = 0;

  // Action communication interface
  virtual void send_goal_request(std::shared_ptr<void> request, ResponseCallback callback) = 0;
  virtual void send_result_request(std::shared_ptr<void> request, ResponseCallback callback) = 0;
  virtual void send_cancel_request(std::shared_ptr<void> request, ResponseCallback callback) = 0;

  // Response handling interface
  virtual void handle_goal_response(const rmw_request_id_t & response_header, std::shared_ptr<void> goal_response) = 0;
  virtual void handle_result_response(const rmw_request_id_t & response_header, std::shared_ptr<void> result_response) = 0;
  virtual void handle_cancel_response(const rmw_request_id_t & response_header, std::shared_ptr<void> cancel_response) = 0;

  // Message creation interface
  virtual std::shared_ptr<void> create_goal_response() const = 0;
  virtual std::shared_ptr<void> create_result_response() const = 0;
  virtual std::shared_ptr<void> create_cancel_response() const = 0;
  virtual std::shared_ptr<void> create_feedback_message() const = 0;
  virtual std::shared_ptr<void> create_status_message() const = 0;

  // Message handling interface
  virtual void handle_feedback_message(std::shared_ptr<void> message) = 0;
  virtual void handle_status_message(std::shared_ptr<void> message) = 0;

  // Waitable interface - provide default implementations
  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override {}
  bool is_ready(const rcl_wait_set_t & wait_set) override { return false; }
  std::shared_ptr<void> take_data() override { return nullptr; }
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override { return nullptr; }
  void execute(const std::shared_ptr<void> & data) override {}

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override {}
  void clear_on_ready_callback() override {}

  // Helper for convenience
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    return wait_for_action_server_nanoseconds(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }
};

} // namespace rclcpp_action