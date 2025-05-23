#pragma once

#include <gmock/gmock.h>
#include <rtest/static_registry.hpp>
#include <rtest/action_server_base.hpp>

#define TEST_TOOLS_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    auto ptr = std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
    ptr->post_init_setup(); \
    return ptr; \
  }

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  TEST_TOOLS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

namespace rclcpp_action {

template<typename ActionT>
class Server : public ServerBase,
              public std::enable_shared_from_this<Server<ActionT>> {
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;

  // using GoalCallback = std::function<GoalResponse::SharedPtr(
  //   const GoalUUID &, std::shared_ptr<const Goal>)>;
  // using CancelCallback = std::function<CancelResponse::SharedPtr(
  //   const std::shared_ptr<GoalHandle>)>;
  // using AcceptedCallback = std::function<void(const std::shared_ptr<GoalHandle>)>;
  /// Signature of a callback that accepts or rejects goal requests.
  using GoalCallback = std::function<GoalResponse(
        const GoalUUID &,
        std::shared_ptr<const typename ActionT::Goal>)>;
  /// Signature of a callback that accepts or rejects requests to cancel a goal.
  using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  /// Signature of a callback that is used to notify when the goal has been accepted.
  using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;


  Server(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & action_name,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : ServerBase(
      node_base,
      action_name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
      options)
  {
    node_base_ = node_base;
    action_name_ = action_name;
    handle_goal_ = handle_goal;
    handle_cancel_ = handle_cancel;
    handle_accepted_ = handle_accepted;
  }

  void post_init_setup() {
    rtest::StaticMocksRegistry::instance().template registerActionServer<ActionT>(
      node_base_->get_fully_qualified_name(),
      action_name_,
      this->shared_from_this());
  }

  // Forwarding methods for testing
  void publish_feedback(
      const typename ActionT::Feedback& feedback,
      const GoalHandleSharedPtr& goal_handle) {
    if (goal_handle) {
      goal_handle->publish_feedback(feedback);
    }
  }

  void publish_status() {
    ServerBase::publish_status();
  }

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::string action_name_;
  GoalCallback handle_goal_;
  CancelCallback handle_cancel_;
  AcceptedCallback handle_accepted_;
};

} // namespace rclcpp_action

namespace rtest {

template<typename ActionT>
class ActionServerMock : public MockBase {
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;
  using GoalResponse = rclcpp_action::GoalResponse;
  using CancelResponse = rclcpp_action::CancelResponse;

  explicit ActionServerMock(rclcpp_action::ServerBase* server) : server_(server) {}
  ~ActionServerMock() { StaticMocksRegistry::instance().detachMock(server_); }

  TEST_TOOLS_SMART_PTR_DEFINITIONS(ActionServerMock<ActionT>)

  // Core mocks
  MOCK_METHOD(GoalResponse,
              handle_goal,
              (const rclcpp_action::GoalUUID&, std::shared_ptr<const Goal>), ());

  MOCK_METHOD(CancelResponse,
              handle_cancel,
              (const GoalHandleSharedPtr&), ());

  MOCK_METHOD(void,
              handle_accepted,
              (const GoalHandleSharedPtr&), ());

  // Helper methods
  void publish_feedback(
      const typename ActionT::Feedback& feedback,
      const GoalHandleSharedPtr& goal_handle) {
    if (goal_handle) {
      goal_handle->publish_feedback(feedback);
    }
  }

  void succeed(
      const typename ActionT::Result& result,
      const GoalHandleSharedPtr& goal_handle) {
    if (goal_handle) {
      goal_handle->succeed(std::make_shared<typename ActionT::Result>(result));
    }
  }

  void abort(
      const typename ActionT::Result& result,
      const GoalHandleSharedPtr& goal_handle) {
    if (goal_handle) {
      goal_handle->abort(std::make_shared<typename ActionT::Result>(result));
    }
  }

  void canceled(
      const typename ActionT::Result& result,
      const GoalHandleSharedPtr& goal_handle) {
    if (goal_handle) {
      goal_handle->canceled(std::make_shared<typename ActionT::Result>(result));
    }
  }

private:
  rclcpp_action::ServerBase* server_{nullptr};
};

template<typename ActionT>
std::shared_ptr<ActionServerMock<ActionT>> findActionServer(
    const std::string& fullyQualifiedNodeName,
    const std::string& actionName) {
  std::shared_ptr<ActionServerMock<ActionT>> server_mock{};
  auto server_base = StaticMocksRegistry::instance()
                        .getActionServer(fullyQualifiedNodeName, actionName)
                        .lock();

  if (server_base) {
    if (StaticMocksRegistry::instance().getMock(server_base.get()).lock()) {
      std::cerr << "WARNING: ActionServerMock already attached\n";
    } else {
      server_mock = std::make_shared<ActionServerMock<ActionT>>(server_base.get());
      StaticMocksRegistry::instance().attachMock(server_base.get(), server_mock);
    }
  }
  return server_mock;
}

template<typename ActionT, typename NodeT>
std::shared_ptr<ActionServerMock<ActionT>> findActionServer(
    const std::shared_ptr<NodeT>& node,
    const std::string& actionName) {
  return findActionServer<ActionT>(node->get_fully_qualified_name(), actionName);
}

} // namespace rtest