// Copyright 2024 Beam Limited.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @file      static_registry.hpp
// @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
// @date      2024-11-26
//
// @brief     Mock header for ROS 2 static registry.

#pragma once

#include <algorithm>
#include <map>
#include <vector>
#include <memory>
#include <iostream>
#include <typeinfo>

#include <boost/type_index.hpp>

#include <rtest/single_instance.hpp>

namespace rclcpp
{
class PublisherBase;
class SubscriptionBase;
class TimerBase;
class ServiceBase;
class ClientBase;
}  // namespace rclcpp


namespace rclcpp_action {
class ServerBase;
class ClientBase;
}  // namespace rclcpp_action

namespace rtest
{

class MockBase
{
};

/**
 * Whenever the ROS 2 Node creates a Subscriber, Publisher or Timer
 * it is registered in this static registry, so the user can retrieve a handle
 * to the mock object in the test.
 */
class StaticMocksRegistry : SingleInstance<StaticMocksRegistry>
{
public:
  using TopicNameT = std::string;
  using FullyQualifiedNodeNameT = std::string;
  using TopicToPublishersMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::PublisherBase>>;
  using TopicToSubscriptionsMapT = std::map<TopicNameT, std::weak_ptr<rclcpp::SubscriptionBase>>;
  using ServiceNameT = std::string;
  using ServiceToServicesMapT = std::map<ServiceNameT, std::weak_ptr<rclcpp::ServiceBase>>;
  using ServiceToClientsMapT = std::map<ServiceNameT, std::weak_ptr<rclcpp::ClientBase>>;
  using ActionNameT = std::string;
  using ActionToServersMapT = std::map<ActionNameT, std::weak_ptr<rclcpp_action::ServerBase>>;
  using ActionToClientsMapT = std::map<ActionNameT, std::weak_ptr<rclcpp_action::ClientBase>>;

  /**
   * @brief Get the static instance of the Mock Registry.
   *
   * @return StaticMocksRegistry&
   */
  static StaticMocksRegistry & instance() { return theRegistry_; }

  /**
   * @brief Register the newly created Publisher in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @param pub       Newly created Publisher object
   */
  template <typename MessageT>
  void registerPublisher(
    const FullyQualifiedNodeNameT & nodeName,
    const TopicNameT & topicName,
    std::weak_ptr<rclcpp::PublisherBase> pub)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerPublisher<"
                << boost::typeindex::type_id<MessageT>().pretty_name() << ">(\"" << nodeName
                << "\", \"" << topicName << "\")\n";
    }
    registerEntity(publishersRegistry_[nodeName], topicName, pub);
  }

  /**
   * @brief Get list of all publishers created by the selected Node.
   *
   * @param nodeName Fully-qualified Node name
   * @return std::vector<std::weak_ptr<rclcpp::PublisherBase>>
   */
  std::vector<std::weak_ptr<rclcpp::PublisherBase>> getNodePublishers(
    const FullyQualifiedNodeNameT & nodeName)
  {
    std::vector<std::weak_ptr<rclcpp::PublisherBase>> publishers{};
    for (auto [topicName, publisher] : publishersRegistry_[nodeName]) {
      publishers.push_back(publisher);
    }
    return publishers;
  }

  /**
   * @brief Get a publisher created by a selected Node for a particular Topic.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::PublisherBase>
   */
  std::weak_ptr<rclcpp::PublisherBase> getPublisher(
    const FullyQualifiedNodeNameT & nodeName,
    const TopicNameT & topicName)
  {
    return findEntity(publishersRegistry_[nodeName], topicName);
  }

  /**
   * @brief Register the newly created Subscription in the regisrtry.
   * This function shall be used by the rclcpp::Publisher only.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @param sub       Newly created Subscription object
   */
  template <typename MessageT>
  void registerSubscription(
    const FullyQualifiedNodeNameT & nodeName,
    const TopicNameT & topicName,
    std::weak_ptr<rclcpp::SubscriptionBase> sub)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerSubscription<"
                << boost::typeindex::type_id<MessageT>().pretty_name() << ">(\"" << nodeName
                << "\", \"" << topicName << "\")\n";
    }
    registerEntity(subscriptionsRegistry_[nodeName], topicName, sub);
  }

  /**
   * @brief Get list of all subscriptions created by the selected Node.
   *
   * @param nodeName
   * @return std::vector<std::weak_ptr<rclcpp::SubscriptionBase>>
   */
  std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> getNodeSubscriptions(
    const FullyQualifiedNodeNameT & nodeName)
  {
    std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> subscriptions{};
    for (auto [topicName, subscription] : subscriptionsRegistry_[nodeName]) {
      subscriptions.push_back(subscription);
    }
    return subscriptions;
  }

  /**
   * @brief Get a subscription created by a selected Node for a particular Topic.
   *
   * @param nodeName  Fully-qualified Node name
   * @param topicName Topic name
   * @return std::weak_ptr<rclcpp::SubscriptionBase>
   */
  std::weak_ptr<rclcpp::SubscriptionBase> getSubscription(
    const FullyQualifiedNodeNameT & nodeName,
    const TopicNameT & topicName)
  {
    return findEntity(subscriptionsRegistry_[nodeName], topicName);
  }

  /**
   * @brief Register the newly created Timer in the regisrtry.
   * This function shall be used by the rclcpp::create_timer() function only.
   *
   * @param nodeName Fully-qualified Node name
   * @param timer    Newly created Timer object
   * @return true
   * @return false
   */
  bool registerTimer(
    const FullyQualifiedNodeNameT & nodeName,
    std::weak_ptr<rclcpp::TimerBase> timer)
  {
    timersRegistry_[nodeName].push_back(timer);
    return true;
  }

  /**
   * @brief Get list of all Timers created by the selected Node.
   *
   * @param nodeName Fully-qualified Node name
   * @return std::vector<std::weak_ptr<rclcpp::TimerBase>>
   */
  std::vector<std::weak_ptr<rclcpp::TimerBase>> getTimers(const FullyQualifiedNodeNameT & nodeName)
  {
    return findEntity(timersRegistry_, nodeName);
  }

  /**
   * @brief Enable additional verbose logs to trace registry events.
   *
   * @param on
   */
  void enableVerboseLogs(bool on) { verbose_ = on; }

  std::weak_ptr<MockBase> getMock(void * ptr)
  {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      return it->second;
    }
    return {};
  }

  void attachMock(void * ptr, std::weak_ptr<MockBase> mock) { mockRegistry_[ptr] = mock; }

  void detachMock(void * ptr)
  {
    auto it = mockRegistry_.find(ptr);
    if (it != mockRegistry_.end()) {
      mockRegistry_.erase(it);
    }
  }

  /**
   * @brief Register the newly created Service in the registry.
   */
  template <typename ServiceT>
  void registerService(
    const FullyQualifiedNodeNameT & nodeName,
    const ServiceNameT & serviceName,
    std::weak_ptr<rclcpp::ServiceBase> service)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerService<"
                << boost::typeindex::type_id<ServiceT>().pretty_name() << ">(\"" << nodeName
                << "\", \"" << serviceName << "\")\n";
    }
    registerEntity(servicesRegistry_[nodeName], serviceName, service);
  }

  /**
   * @brief Get list of all services created by the selected Node.
   */
  std::vector<std::weak_ptr<rclcpp::ServiceBase>> getNodeServices(
    const FullyQualifiedNodeNameT & nodeName)
  {
    std::vector<std::weak_ptr<rclcpp::ServiceBase>> services{};
    for (auto [serviceName, service] : servicesRegistry_[nodeName]) {
      services.push_back(service);
    }
    return services;
  }

  /**
   * @brief Get a service created by a selected Node.
   */
  std::weak_ptr<rclcpp::ServiceBase> getService(
    const FullyQualifiedNodeNameT & nodeName,
    const ServiceNameT & serviceName)
  {
    return findEntity(servicesRegistry_[nodeName], serviceName);
  }

  template <typename ServiceT>
  void registerServiceClient(
    const FullyQualifiedNodeNameT & nodeName,
    const ServiceNameT & serviceName,
    std::weak_ptr<rclcpp::ClientBase> client)
  {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerServiceClient<"
                << boost::typeindex::type_id<ServiceT>().pretty_name() << ">(\"" << nodeName
                << "\", \"" << serviceName << "\")\n";
    }
    registerEntity(serviceClientsRegistry_[nodeName], serviceName, client);
  }

  std::vector<std::weak_ptr<rclcpp::ClientBase>> getNodeServiceClients(
    const FullyQualifiedNodeNameT & nodeName)
  {
    std::vector<std::weak_ptr<rclcpp::ClientBase>> clients{};
    for (auto [serviceName, client] : serviceClientsRegistry_[nodeName]) {
      clients.push_back(client);
    }
    return clients;
  }

  std::weak_ptr<rclcpp::ClientBase> getServiceClient(
    const FullyQualifiedNodeNameT & nodeName,
    const ServiceNameT & serviceName)
  {
    return findEntity(serviceClientsRegistry_[nodeName], serviceName);
  }

  template <typename ActionT>
  void registerActionServer(
      const FullyQualifiedNodeNameT& nodeName,
      const ActionNameT& actionName,
      std::weak_ptr<rclcpp_action::ServerBase> server) {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerActionServer<"
                << boost::typeindex::type_id<ActionT>().pretty_name()
                << ">(\"" << nodeName << "\", \"" << actionName << "\")\n";
    }
    registerEntity(actionServersRegistry_[nodeName], actionName, server);
  }

  template <typename ActionT>
  void registerActionClient(
      const FullyQualifiedNodeNameT& nodeName,
      const ActionNameT& actionName,
      std::weak_ptr<rclcpp_action::ClientBase> client) {
    if (verbose_) {
      std::cout << "StaticMocksRegistry::registerActionClient<"
                << boost::typeindex::type_id<ActionT>().pretty_name()
                << ">(\"" << nodeName << "\", \"" << actionName << "\")\n";
    }
    registerEntity(actionClientsRegistry_[nodeName], actionName, client);
  }

  std::weak_ptr<rclcpp_action::ServerBase> getActionServer(
      const FullyQualifiedNodeNameT& nodeName,
      const ActionNameT& actionName) {
    return findEntity(actionServersRegistry_[nodeName], actionName);
  }

  std::weak_ptr<rclcpp_action::ClientBase> getActionClient(
      const FullyQualifiedNodeNameT& nodeName,
      const ActionNameT& actionName) {
    return findEntity(actionClientsRegistry_[nodeName], actionName);
  }

private:
  StaticMocksRegistry() {}

  static StaticMocksRegistry theRegistry_;

  template <typename RegistryT, typename EntityT>
  void registerEntity(RegistryT & reg, const TopicNameT & topicName, EntityT e)
  {
    if (!reg[topicName].lock()) {
      reg[topicName] = e;
    }
  }

  template <typename RegistryT>
  typename RegistryT::value_type::second_type findEntity(
    RegistryT & reg,
    const TopicNameT & topicName)
  {
    auto it = reg.find(topicName);
    if (it != reg.end()) {
      return it->second;
    } else {
      return {};
    }
  }

  std::map<FullyQualifiedNodeNameT, TopicToPublishersMapT> publishersRegistry_;
  std::map<FullyQualifiedNodeNameT, TopicToSubscriptionsMapT> subscriptionsRegistry_;
  std::map<FullyQualifiedNodeNameT, std::vector<std::weak_ptr<rclcpp::TimerBase>>> timersRegistry_;
  std::map<FullyQualifiedNodeNameT, ServiceToServicesMapT> servicesRegistry_;
  std::map<FullyQualifiedNodeNameT, ServiceToClientsMapT> serviceClientsRegistry_;
  std::map<FullyQualifiedNodeNameT, ActionToServersMapT> actionServersRegistry_;
  std::map<FullyQualifiedNodeNameT, ActionToClientsMapT> actionClientsRegistry_;

  std::map<void *, std::weak_ptr<MockBase>> mockRegistry_;

  bool verbose_{false};
};

void enableVerboseLogs(bool on);

}  // namespace rtest
