#include "hdl_localization/globalmap_server_node.hpp"

namespace hdl_localization {

GlobalmapServerNode::GlobalmapServerNode(const rclcpp::NodeOptions &options)
    : Node("globalmap_server_node", options) {

  initialize_params();

  // publish globalmap with "latched" publisher
  globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/globalmap", rclcpp::QoS(5).transient_local());
  map_update_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/map_request/pcd", 10,
      std::bind(&GlobalmapServerNode::map_update_callback, this,
                std::placeholders::_1));

  globalmap_pub_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalmapServerNode::pub_once_cb, this));

  RCLCPP_INFO(this->get_logger(), "GlobalmapServerNode initialized");
}

void GlobalmapServerNode::initialize_params() {
  // read globalmap from a pcd file
  std::string globalmap_pcd =
      this->declare_parameter<std::string>("globalmap_pcd", "");
  globalmap_.reset(new pcl::PointCloud<PointT>());

  if (globalmap_pcd.find(".ply") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "Loading globalmap from ply file");
    pcl::io::loadPLYFile(globalmap_pcd, *globalmap_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Loading globalmap from pcd file");
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap_);
  }
  globalmap_->header.frame_id = "map";

  std::ifstream utm_file(globalmap_pcd + ".utm");
  if (utm_file.is_open() &&
      this->declare_parameter<bool>("convert_utm_to_local", true)) {
    double utm_easting;
    double utm_northing;
    double altitude;
    utm_file >> utm_easting >> utm_northing >> altitude;
    for (auto &pt : globalmap_->points) {
      pt.getVector3fMap() -=
          Eigen::Vector3f(utm_easting, utm_northing, altitude);
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Global map offset by UTM reference coordinates (x = "
                           << utm_easting << ", y = " << utm_northing
                           << ") and altitude (z = " << altitude << ")");
  }

  // downsample globalmap
  double downsample_resolution =
      this->declare_parameter<double>("downsample_resolution", 0.1);
  boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(
      new pcl::VoxelGrid<PointT>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                         downsample_resolution);
  voxelgrid->setInputCloud(globalmap_);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  voxelgrid->filter(*filtered);

  globalmap_ = filtered;
}

void GlobalmapServerNode::pub_once_cb() {
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*globalmap_, *message);
  message->header.frame_id = globalmap_->header.frame_id;
  RCLCPP_INFO(this->get_logger(), "Publishing globalmap");
  RCLCPP_INFO(this->get_logger(), "Number of points: %ld",
              globalmap_->points.size());
  globalmap_pub_->publish(*message);
  globalmap_pub_timer_->cancel(); // Only publish once at startup
}

void GlobalmapServerNode::map_update_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received map request, map path : " << msg->data);
  std::string globalmap_pcd = msg->data;
  globalmap_.reset(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(globalmap_pcd, *globalmap_);
  globalmap_->header.frame_id = "map";

  // downsample globalmap
  double downsample_resolution =
      this->declare_parameter<double>("downsample_resolution", 0.1);
  boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(
      new pcl::VoxelGrid<PointT>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                         downsample_resolution);
  voxelgrid->setInputCloud(globalmap_);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  voxelgrid->filter(*filtered);

  globalmap_   = filtered;
  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*globalmap_, *message);
  message->header.frame_id = globalmap_->header.frame_id;
  globalmap_pub_->publish(*message);
}

} // namespace hdl_localization

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerNode)