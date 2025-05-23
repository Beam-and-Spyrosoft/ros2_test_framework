
// clang-format off
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/event_callback.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/waitable.hpp"

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"


namespace rclcpp_action
{
// Forward declaration
class ClientBaseImpl;

/// Base Action Client implementation
/// \internal
/**
 * This class should not be used directly by users wanting to create an aciton client.
 * Instead users should use `rclcpp_action::Client<>`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ClientBase : public rclcpp::Waitable
{
public:
  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  /// Enum to identify entities belonging to the action client
  enum class EntityType : std::size_t
  {
    GoalClient,
    ResultClient,
    CancelClient,
    FeedbackSubscription,
    StatusSubscription,
  };

  /// Return true if there is an action server that is ready to take goal requests.
  RCLCPP_ACTION_PUBLIC
  bool
  action_server_is_ready() const;

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_action_server(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_action_server_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  // -------------
  // Waitables API

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_subscriptions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(const rcl_wait_set_t & wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute(const std::shared_ptr<void> & data) override;

  /// \internal
  /// Set a callback to be called when action client entities have an event
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * The callback also receives an int identifier argument, which identifies
   * the action client entity which is ready.
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \param[in] callback functor to be called when a new message is received.
   */
  RCLCPP_ACTION_PUBLIC
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /// Unset the callback registered for new events, if any.
  RCLCPP_ACTION_PUBLIC
  void
  clear_on_ready_callback() override;

  // End Waitables API
  // -----------------

protected:
  RCLCPP_ACTION_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & options);

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  RCLCPP_ACTION_PUBLIC
  bool
  wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout);

  // -----------------------------------------------------
  // API for communication between ClientBase and Client<>
  using ResponseCallback = std::function<void (std::shared_ptr<void> response)>;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  rclcpp::Logger get_logger();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  generate_goal_id();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_goal_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_result_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_cancel_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_goal_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_goal_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> goal_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_result_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_result_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> result_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_cancel_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_cancel_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> cancel_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_feedback_message() const = 0;

  /// \internal
  virtual
  void
  handle_feedback_message(std::shared_ptr<void> message) = 0;

  /// \internal
  virtual
  std::shared_ptr<void>
  create_status_message() const = 0;

  /// \internal
  virtual
  void
  handle_status_message(std::shared_ptr<void> message) = 0;

  // End API for communication between ClientBase and Client<>
  // ---------------------------------------------------------

  /// \internal
  /// Set a callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_on_ready_callback(
    EntityType entity_type,
    rcl_event_callback_t callback,
    const void * user_data);

  // Mutex to protect the callbacks storage.
  std::recursive_mutex listener_mutex_;
  // Storage for std::function callbacks to keep them in scope
  std::unordered_map<EntityType, std::function<void(size_t)>> entity_type_to_on_ready_callback_;

private:
  std::unique_ptr<ClientBaseImpl> pimpl_;

  /// Set a std::function callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_callback_to_entity(
    EntityType entity_type,
    std::function<void(size_t, int)> callback);

  bool on_ready_callback_set_{false};
};

}  // namespace rclcpp_action
// clang-format on