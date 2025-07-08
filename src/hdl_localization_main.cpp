#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "hdl_localization/globalmap_server_node.hpp"
#include "hdl_localization/hdl_localization_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions                      options;

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Starting HDL Localization main executable");

  try {
    auto globalmap_node =
        std::make_shared<hdl_localization::GlobalmapServerNode>(options);
    auto localization_node =
        std::make_shared<hdl_localization::HdlLocalizationNode>(options);

    // Add both nodes to executor
    executor.add_node(globalmap_node);
    executor.add_node(localization_node);
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Both components started successfully");

    // Spin the executor
    executor.spin();

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}