#pragma once

#include <memory>
#include <functional>
#include <chrono>
#include <future>
#include "rclcpp/waitable.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action {

enum class ResultCode : int8_t {
  UNKNOWN = 0,
  SUCCEEDED = 1,
  CANCELED = 2,
  ABORTED = 3
};

class ClientBase : public rclcpp::Waitable {
public:
  ClientBase() = default;
  virtual ~ClientBase() = default;

  // Basic action client interface
  virtual bool action_server_is_ready() const { return true; }
  virtual rclcpp::Logger get_logger() { return rclcpp::get_logger("action_client_mock"); }

  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override { (void)wait_set; }
  bool is_ready(const rcl_wait_set_t & wait_set) override { (void)wait_set; return false; }
  std::shared_ptr<void> take_data() override { return nullptr; }
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override { (void)id; return nullptr; }
  void execute(const std::shared_ptr<void> & data) override { (void)data; }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override { (void)callback; }
  void clear_on_ready_callback() override {}

  // Helper for convenience
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool wait_for_action_server(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1)) {
    (void)timeout;
    return true;
  }
};

} // namespace rclcpp_action