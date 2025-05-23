#pragma once
#include <gmock/gmock.h>
#include <rtest/static_registry.hpp>
#include <rtest/action_client_base.hpp>
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include <future>

#define TEST_TOOLS_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> make_shared(Args && ... args) { \
    auto ptr = std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
    ptr->post_init_setup(); return ptr; }

#define TEST_TOOLS_SMART_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __RCLCPP_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __RCLCPP_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  TEST_TOOLS_MAKE_SHARED_DEFINITION(__VA_ARGS__)

namespace rclcpp_action {
template<typename ActionT>
class ClientGoalHandle {
public:
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using Goal = typename ActionT::Goal;
  using SharedPtr = std::shared_ptr<ClientGoalHandle>;

  struct WrappedResult {
    ResultCode code;
    std::shared_ptr<Result> result;
  };

  using FeedbackCallback = std::function<void(SharedPtr, std::shared_ptr<const Feedback>)>;
  using ResultCallback = std::function<void(const WrappedResult&)>;

  FeedbackCallback feedback_callback;
  ResultCallback result_callback;
};

template<typename ActionT>
class Client : public ClientBase, public std::enable_shared_from_this<Client<ActionT>> {
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void(GoalHandleSharedPtr)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;

  struct SendGoalOptions {
    SendGoalOptions() : goal_response_callback(nullptr), feedback_callback(nullptr), result_callback(nullptr) {}
    GoalResponseCallback goal_response_callback;
    FeedbackCallback feedback_callback;
    ResultCallback result_callback;
  };

  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & action_name)
  : node_base_(node_base), action_name_(action_name) {
    (void)node_graph;
  }

  void post_init_setup() {
    rtest::StaticMocksRegistry::instance().template registerActionClient<ActionT>(
      node_base_->get_fully_qualified_name(), action_name_, this->shared_from_this());
  }

  std::shared_future<GoalHandleSharedPtr> async_send_goal(const Goal& goal, const SendGoalOptions& options) {
    (void)goal; (void)options;
    std::promise<GoalHandleSharedPtr> promise;
    promise.set_value(nullptr);
    return promise.get_future().share();
  }

  std::shared_future<WrappedResult> async_get_result(const GoalHandleSharedPtr& goal_handle) {
    (void)goal_handle;
    std::promise<WrappedResult> promise;
    WrappedResult result;
    result.code = ResultCode::SUCCEEDED;
    result.result = std::make_shared<Result>();
    promise.set_value(result);
    return promise.get_future().share();
  }

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::string action_name_;
};
} // namespace rclcpp_action

namespace rtest {
template<typename ActionT>
class ActionClientMock : public MockBase {
public:
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using SendGoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;

  explicit ActionClientMock(rclcpp_action::ClientBase* client) : client_(client) {}
  ~ActionClientMock() { StaticMocksRegistry::instance().detachMock(client_); }
  TEST_TOOLS_SMART_PTR_DEFINITIONS(ActionClientMock<ActionT>)

  MOCK_METHOD(std::shared_future<GoalHandleSharedPtr>, async_send_goal, (const Goal&, const SendGoalOptions&), ());
  MOCK_METHOD(std::shared_future<WrappedResult>, async_get_result, (const GoalHandleSharedPtr&), ());
  MOCK_METHOD(bool, action_server_is_ready, (), ());

  void simulate_feedback(const GoalHandleSharedPtr& goal_handle, const typename ActionT::Feedback& feedback) {
    if (goal_handle && goal_handle->feedback_callback) {
      auto feedback_msg = std::make_shared<const typename ActionT::Feedback>(feedback);
      goal_handle->feedback_callback(goal_handle, feedback_msg);
    }
  }

  void simulate_result(const GoalHandleSharedPtr& goal_handle, const WrappedResult& result) {
    if (goal_handle && goal_handle->result_callback) {
      goal_handle->result_callback(result);
    }
  }

private:
  rclcpp_action::ClientBase* client_{nullptr};
};

template<typename ActionT>
std::shared_ptr<ActionClientMock<ActionT>> findActionClient(const std::string& fullyQualifiedNodeName, const std::string& actionName) {
  std::shared_ptr<ActionClientMock<ActionT>> client_mock{};
  auto client_base = StaticMocksRegistry::instance().getActionClient(fullyQualifiedNodeName, actionName).lock();
  if (client_base) {
    if (StaticMocksRegistry::instance().getMock(client_base.get()).lock()) {
      std::cerr << "WARNING: ActionClientMock already attached\n";
    } else {
      client_mock = std::make_shared<ActionClientMock<ActionT>>(client_base.get());
      StaticMocksRegistry::instance().attachMock(client_base.get(), client_mock);
    }
  }
  return client_mock;
}

template<typename ActionT, typename NodeT>
std::shared_ptr<ActionClientMock<ActionT>> findActionClient(const std::shared_ptr<NodeT>& node, const std::string& actionName) {
  return findActionClient<ActionT>(node->get_fully_qualified_name(), actionName);
}
} // namespace rtest