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
  ServerBase() = default;
  virtual ~ServerBase() = default;

  size_t get_number_of_ready_subscriptions() override { return 0; }
  size_t get_number_of_ready_timers() override { return 0; }
  size_t get_number_of_ready_clients() override { return 0; }
  size_t get_number_of_ready_services() override { return 0; }
  size_t get_number_of_ready_guard_conditions() override { return 0; }

  void add_to_wait_set(rcl_wait_set_t & wait_set) override { (void)wait_set; }
  bool is_ready(const rcl_wait_set_t & wait_set) override { (void)wait_set; return false; }
  std::shared_ptr<void> take_data() override { return nullptr; }
  std::shared_ptr<void> take_data_by_entity_id(size_t id) override { (void)id; return nullptr; }
  void execute(const std::shared_ptr<void> & data) override { (void)data; }

  void set_on_ready_callback(std::function<void(size_t, int)> callback) override { (void)callback; }
  void clear_on_ready_callback() override {}

  virtual void publish_status() {}
};

} // namespace rclcpp_action