// clang-format off
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "action_msgs/srv/cancel_goal.hpp"
#include "rcl/event_callback.h"
#include "rcl_action/action_server.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/waitable.hpp"

#include "rclcpp_action/visibility_control.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"

namespace rclcpp_action
{
// Forward declaration
class ServerBaseImpl;

/// A response returned by an action server callback when a goal is requested.
enum class GoalResponse : int8_t
{
  /// The goal is rejected and will not be executed.
  REJECT = 1,
  /// The server accepts the goal, and is going to begin execution immediately.
  ACCEPT_AND_EXECUTE = 2,
  /// The server accepts the goal, and is going to execute it later.
  ACCEPT_AND_DEFER = 3,
};

/// A response returned by an action server callback when a goal has been asked to be canceled.
enum class CancelResponse : int8_t
{
  /// The server will not try to cancel the goal.
  REJECT = 1,
  /// The server has agreed to try to cancel the goal.
  ACCEPT = 2,
};

/// Base Action Server implementation
/// \internal
/**
 * This class should not be used directly by users writing an action server.
 * Instead users should use `rclcpp_action::Server`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ServerBase : public rclcpp::Waitable
{
public:
  /// Enum to identify entities belonging to the action server
  enum class EntityType : std::size_t
  {
    GoalService,
    ResultService,
    CancelService,
    Expired,
  };

  RCLCPP_ACTION_PUBLIC
  virtual ~ServerBase();

  // -------------
  // Waitables API

  /// Return the number of subscriptions used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_subscriptions() override;

  /// Return the number of timers used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  /// Return the number of service clients used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  /// Return the number of service servers used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  /// Return the number of guard conditions used to implement an action server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  /// Add all entities to a wait set.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t & wait_set) override;

  /// Return true if any entity belonging to the action server is ready to be executed.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(const rcl_wait_set_t & wait_set) override;

  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data() override;

  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override;

  /// Act on entities in the wait set which are ready to be acted upon.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute(const std::shared_ptr<void> & data) override;

  /// \internal
  /// Set a callback to be called when action server entities have an event
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * The callback also receives an int identifier argument, which identifies
   * the action server entity which is ready.
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

  /// Unset the callback to be called whenever the waitable becomes ready.
  RCLCPP_ACTION_PUBLIC
  void
  clear_on_ready_callback() override;

  // End Waitables API
  // -----------------

protected:
  RCLCPP_ACTION_PUBLIC
  ServerBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_server_options_t & options);

  // -----------------------------------------------------
  // API for communication between ServerBase and Server<>

  // ServerBase will call this function when a goal request is received.
  // The subclass should convert to the real type and call a user's callback.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::pair<GoalResponse, std::shared_ptr<void>>
  call_handle_goal_callback(GoalUUID &, std::shared_ptr<void> request) = 0;

  // ServerBase will determine which goal ids are being cancelled, and then call this function for
  // each goal id.
  // The subclass should look up a goal handle and call the user's callback.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  CancelResponse
  call_handle_cancel_callback(const GoalUUID & uuid) = 0;

  /// Given a goal request message, return the UUID contained within.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  get_goal_id_from_goal_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_goal_request() = 0;

  /// Call user callback to inform them a goal has been accepted.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  call_goal_accepted_callback(
    std::shared_ptr<rcl_action_goal_handle_t> rcl_goal_handle,
    GoalUUID uuid, std::shared_ptr<void> goal_request_message) = 0;

  /// Given a result request message, return the UUID contained within.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  get_goal_id_from_result_request(void * message) = 0;

  /// Create an empty goal request message so it can be taken from a lower layer.
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_result_request() = 0;

  /// Create an empty goal result message so it can be sent as a reply in a lower layer
  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  std::shared_ptr<void>
  create_result_response(decltype(action_msgs::msg::GoalStatus::status) status) = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_status();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  notify_goal_terminal_state();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_result(const GoalUUID & uuid, std::shared_ptr<void> result_msg);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  publish_feedback(std::shared_ptr<void> feedback_msg);

  // End API for communication between ServerBase and Server<>
  // ---------------------------------------------------------

private:
  /// Handle a request to add a new goal to the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_goal_request_received(
    rcl_ret_t ret,
    rcl_action_goal_info_t goal_info,
    rmw_request_id_t request_header,
    std::shared_ptr<void> message);

  /// Handle a request to cancel goals on the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_cancel_request_received(
    rcl_ret_t ret,
    std::shared_ptr<action_msgs::srv::CancelGoal::Request> request,
    rmw_request_id_t request_header);

  /// Handle a request to get the result of an action
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_result_request_received(
    rcl_ret_t ret,
    std::shared_ptr<void> result_request,
    rmw_request_id_t request_header);

  /// Handle a timeout indicating a completed goal should be forgotten by the server
  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute_check_expired_goals();

  /// Private implementation
  /// \internal
  std::unique_ptr<ServerBaseImpl> pimpl_;

  /// Set a std::function callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_callback_to_entity(
    EntityType entity_type,
    std::function<void(size_t, int)> callback);

protected:
  // Mutex to protect the callbacks storage.
  std::recursive_mutex listener_mutex_;
  // Storage for std::function callbacks to keep them in scope
  std::unordered_map<EntityType, std::function<void(size_t)>> entity_type_to_on_ready_callback_;

  /// Set a callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_on_ready_callback(
    EntityType entity_type,
    rcl_event_callback_t callback,
    const void * user_data);

  bool on_ready_callback_set_{false};
};
}  // namespace rclcpp_action
// clang-format on
