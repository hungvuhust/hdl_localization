#ifndef HDL_LOCALIZATION__GLOBALMAP_SERVER_NODE_HPP_
#define HDL_LOCALIZATION__GLOBALMAP_SERVER_NODE_HPP_

#include <iostream>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hdl_localization {

class GlobalmapServerNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  explicit GlobalmapServerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void initialize_params();
  void pub_once_cb();
  void map_update_callback(const std_msgs::msg::String::SharedPtr msg);

private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      map_update_sub_;

  rclcpp::TimerBase::SharedPtr globalmap_pub_timer_;
  pcl::PointCloud<PointT>::Ptr globalmap_;
};

} // namespace hdl_localization

#endif // HDL_LOCALIZATION__GLOBALMAP_SERVER_NODE_HPP_