#pragma once
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace utils {

template <class ServiceT> class ServiceClient {
public:
  using Ptr = std::shared_ptr<ServiceClient<ServiceT>>;

  explicit ServiceClient(const std::string             &service_name,
                         const rclcpp::Node::SharedPtr &provided_node)
      : service_name_(service_name), node_(provided_node) {
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());
    client_ = node_->create_client<ServiceT>(
        service_name, rclcpp::ServicesQoS().get_rmw_qos_profile(),
        callback_group_);
  }

  using RequestType  = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  typename ResponseType::SharedPtr invoke(
      typename RequestType::SharedPtr &request,
      const std::chrono::nanoseconds   timeout = std::chrono::nanoseconds(-1)) {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
            service_name_ +
            " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(node_->get_logger(),
                  "%s service client: waiting for service to appear...",
                  service_name_.c_str());
    }

    RCLCPP_DEBUG(node_->get_logger(), "%s service client: send async request",
                 service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_.spin_until_future_complete(
            future_result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
      // Pending request must be manually cleaned up if execution is interrupted
      // or timed out
      client_->remove_pending_request(future_result);
      throw std::runtime_error(service_name_ +
                               " service client: async_send_request failed");
    }

    return future_result.get();
  }

  bool invoke(typename RequestType::SharedPtr  &request,
              typename ResponseType::SharedPtr &response) {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error(
            service_name_ +
            " service client: interrupted while waiting for service");
      }
      RCLCPP_INFO(node_->get_logger(),
                  "%s service client: waiting for service to appear...",
                  service_name_.c_str());
    }

    RCLCPP_DEBUG(node_->get_logger(), "%s service client: send async request",
                 service_name_.c_str());
    auto future_result = client_->async_send_request(request);

    if (callback_group_executor_.spin_until_future_complete(future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      // Pending request must be manually cleaned up if execution is interrupted
      // or timed out
      client_->remove_pending_request(future_result);
      return false;
    }

    response = future_result.get();
    return response.get();
  }

  bool wait_for_service(const std::chrono::nanoseconds timeout =
                            std::chrono::nanoseconds::max()) {
    return client_->wait_for_service(timeout);
  }

  std::string getServiceName() { return service_name_; }

protected:
  std::string                                  service_name_;
  rclcpp::Node::SharedPtr                      node_;
  rclcpp::CallbackGroup::SharedPtr             callback_group_;
  rclcpp::executors::SingleThreadedExecutor    callback_group_executor_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

} // namespace utils
