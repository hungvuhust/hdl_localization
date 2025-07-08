#ifndef HDL_LOCALIZATION__HDL_LOCALIZATION_NODE_HPP_
#define HDL_LOCALIZATION__HDL_LOCALIZATION_NODE_HPP_

#include <iostream>
#include <memory>
#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <pclomp/ndt_omp.h>

#include <hdl_localization/delta_estimater.hpp>
#include <hdl_localization/pose_estimator.hpp>

#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_localization/msg/scan_matching_status.hpp>

namespace hdl_localization {

class HdlLocalizationNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  explicit HdlLocalizationNode(const rclcpp::NodeOptions &options);

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const;
  void                                   initialize_params();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void
  points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg);
  void
  globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg);
  bool relocalize(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                  std::shared_ptr<std_srvs::srv::Empty::Response>      res);
  void initialpose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);
  pcl::PointCloud<PointT>::ConstPtr
       downsample(const pcl::PointCloud<PointT>::ConstPtr &cloud) const;
  void publish_odometry(const rclcpp::Time &stamp, const Eigen::Matrix4f &pose);
  void publish_scan_matching_status(
      const std_msgs::msg::Header             &header,
      const pcl::PointCloud<PointT>::ConstPtr &aligned);

private:
  // Registration parameters
  mutable std::string reg_method_;
  mutable std::string ndt_neighbor_search_method_;
  mutable double      ndt_neighbor_search_radius_;
  mutable double      ndt_resolution_;

  // ROS
  std::string robot_odom_frame_id_;
  std::string odom_child_frame_id_;

  bool                                                           use_imu_;
  bool                                                           invert_acc_;
  bool                                                           invert_gyro_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr
      status_pub_;

  tf2_ros::Buffer               tf_buffer_;
  tf2_ros::TransformListener    tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // imu input buffer
  std::mutex                                    imu_data_mutex_;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_data_;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr           globalmap_;
  pcl::Filter<PointT>::Ptr               downsample_filter_;
  pcl::Registration<PointT, PointT>::Ptr registration_;
  pcl::search::KdTree<PointT>            globalmap_search_;

  // pose estimator
  std::mutex                                       pose_estimator_mutex_;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator_;

  // global localization
  bool                            use_global_localization_;
  std::atomic_bool                relocalizing_;
  std::unique_ptr<DeltaEstimater> delta_estimater_;

  pcl::PointCloud<PointT>::ConstPtr                last_scan_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server_;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr
      set_global_map_client_;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::
      SharedPtr query_global_localization_client_;
};

} // namespace hdl_localization

#endif // HDL_LOCALIZATION__HDL_LOCALIZATION_NODE_HPP_