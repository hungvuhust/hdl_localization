#include "hdl_localization/hdl_localization_node.hpp"
#include "fast_gicp/gicp/fast_gicp.hpp"
#include "fast_gicp/gicp/fast_vgicp.hpp"
#include "rclcpp/callback_group.hpp"
#include <rclcpp/qos.hpp>

namespace hdl_localization {

HdlLocalizationNode::HdlLocalizationNode(const rclcpp::NodeOptions &options)
    : Node("hdl_localization_node", options), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_), tf_broadcaster_(this) {

  node_ = std::make_shared<rclcpp::Node>("hdl_localization_node_cli");
  initialize_params();

  robot_odom_frame_id_ =
      this->declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
  odom_child_frame_id_ =
      this->declare_parameter<std::string>("odom_child_frame_id", "base_link");

  use_imu_     = this->declare_parameter<bool>("use_imu", true);
  invert_acc_  = this->declare_parameter<bool>("invert_acc", false);
  invert_gyro_ = this->declare_parameter<bool>("invert_gyro", false);

  if (use_imu_) {
    RCLCPP_INFO(this->get_logger(), "enable imu-based prediction");
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 256,
        std::bind(&HdlLocalizationNode::imu_callback, this,
                  std::placeholders::_1));
  }

  points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, 5,
      std::bind(&HdlLocalizationNode::points_callback, this,
                std::placeholders::_1));

  globalmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/globalmap", 1,
      std::bind(&HdlLocalizationNode::globalmap_callback, this,
                std::placeholders::_1));

  initialpose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::QoS(10),
          std::bind(&HdlLocalizationNode::initialpose_callback, this,
                    std::placeholders::_1));

  pose_pub_    = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
  aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/aligned_points", 5);
  status_pub_ =
      this->create_publisher<hdl_localization::msg::ScanMatchingStatus>(
          "/status", 5);

  // global localization
  use_global_localization_ =
      this->get_parameter("use_global_localization").as_bool();
  if (use_global_localization_) {
    RCLCPP_INFO(this->get_logger(), "wait for global localization services");

    set_global_map_client_ = std::make_shared<
        utils::ServiceClient<hdl_global_localization::srv::SetGlobalMap>>(
        "/hdl_global_localization/set_global_map", node_);
    query_global_localization_client_ = std::make_shared<utils::ServiceClient<
        hdl_global_localization::srv::QueryGlobalLocalization>>(
        "/hdl_global_localization/query", node_);

    relocalize_server_ = node_->create_service<std_srvs::srv::Empty>(
        "/relocalize", std::bind(&HdlLocalizationNode::relocalize, this,
                                 std::placeholders::_1, std::placeholders::_2));
  }

  client_thread_ = std::make_shared<std::thread>([this]() {
    rclcpp::spin(node_);
    RCLCPP_INFO(this->get_logger(), "client thread exited");
  });
  client_thread_->detach();
}

pcl::Registration<HdlLocalizationNode::PointT, HdlLocalizationNode::PointT>::Ptr
HdlLocalizationNode::create_registration() {

  reg_method_ = this->get_parameter("reg_method").as_string();
  ndt_neighbor_search_method_ =
      this->get_parameter("ndt_neighbor_search_method").as_string();
  ndt_neighbor_search_radius_ =
      this->get_parameter("ndt_neighbor_search_radius").as_double();
  ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();

  if (reg_method_ == "NDT_OMP") {
    RCLCPP_INFO(this->get_logger(), "NDT_OMP is selected");
    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution_);
    if (ndt_neighbor_search_method_ == "DIRECT1") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if (ndt_neighbor_search_method_ == "DIRECT7") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if (ndt_neighbor_search_method_ == "KDTREE") {
        RCLCPP_INFO(this->get_logger(), "search_method KDTREE is selected");
      } else {
        RCLCPP_WARN(this->get_logger(), "invalid search method was given");
        RCLCPP_WARN(this->get_logger(), "default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    return ndt;
  }
#if 0
  else if (reg_method_.find("NDT_CUDA") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "NDT_CUDA is selected");
    std::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(
        new fast_gicp::NDTCuda<PointT, PointT>);
    ndt->setResolution(ndt_resolution_);

    if (reg_method_.find("D2D") != std::string::npos) {
      ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    } else if (reg_method_.find("P2D") != std::string::npos) {
      ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    }

    if (ndt_neighbor_search_method_ == "DIRECT1") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    } else if (ndt_neighbor_search_method_ == "DIRECT7") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    } else if (ndt_neighbor_search_method_ == "DIRECT_RADIUS") {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "search_method DIRECT_RADIUS is selected : "
                             << ndt_neighbor_search_radius_);
      ndt->setNeighborSearchMethod(
          fast_gicp::NeighborSearchMethod::DIRECT_RADIUS,
          ndt_neighbor_search_radius_);
    } else {
      RCLCPP_WARN(this->get_logger(), "invalid search method was given");
    }
    return ndt;
  }
#endif
  else if (reg_method_.find("VGICP") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "VGICP is selected");
    std::shared_ptr<fast_gicp::FastVGICP<PointT, PointT>> ndt(
        new fast_gicp::FastVGICP<PointT, PointT>);
    ndt->setResolution(ndt_resolution_);

    if (ndt_neighbor_search_method_ == "DIRECT1") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    } else if (ndt_neighbor_search_method_ == "DIRECT7") {
      RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    } else {
      RCLCPP_WARN(this->get_logger(), "invalid search method was given");
    }
    return ndt;
  } else if (reg_method_.find("GICP") != std::string::npos) {
    RCLCPP_INFO(this->get_logger(), "GICP is selected");
    std::shared_ptr<fast_gicp::FastGICP<PointT, PointT>> ndt(
        new fast_gicp::FastGICP<PointT, PointT>);
    ndt->setNumThreads(4);
    ndt->setCorrespondenceRandomness(20);
    ndt->setTransformationEpsilon(1e-6);
    ndt->setMaximumIterations(100);
    return ndt;
  }

  RCLCPP_ERROR_STREAM(this->get_logger(),
                      "unknown registration method:" << reg_method_);
  return nullptr;
}
//
void HdlLocalizationNode::initialize_params() {
  this->declare_parameter<std::string>("reg_method", "NDT_CUDA");
  this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT1");
  this->declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
  this->declare_parameter<double>("ndt_resolution", 1.0);
  this->declare_parameter<double>("downsample_resolution", 0.1);
  this->declare_parameter<bool>("specify_init_pose", true);
  this->declare_parameter<double>("init_pos_x", 0.0);
  this->declare_parameter<double>("init_pos_y", 0.0);
  this->declare_parameter<double>("init_pos_z", 0.0);
  this->declare_parameter<double>("init_ori_w", 1.0);
  this->declare_parameter<double>("init_ori_x", 0.0);
  this->declare_parameter<double>("init_ori_y", 0.0);
  this->declare_parameter<double>("init_ori_z", 0.0);
  this->declare_parameter<std::string>("points_topic", "/livox/lidar");
  this->declare_parameter<std::string>("imu_topic", "/livox/imu");
  this->declare_parameter<bool>("use_global_localization", true);
  this->declare_parameter<bool>("enable_robot_odometry_prediction", false);
  this->declare_parameter<double>("cool_time_duration", 0.5);
  this->declare_parameter<double>("status_max_correspondence_dist", 0.5);
  this->declare_parameter<double>("status_max_valid_point_dist", 25.0);

  // initialize parameters
  RCLCPP_INFO(this->get_logger(), "--------------------------------");
  RCLCPP_INFO(this->get_logger(), "HDL Localization Node Parameters:");
  RCLCPP_INFO(this->get_logger(), "--------------------------------");

  // initialize scan matching method
  double downsample_resolution =
      this->get_parameter("downsample_resolution").as_double();

  pcl::VoxelGrid<PointT>::Ptr voxelgrid(new pcl::VoxelGrid<PointT>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution,
                         downsample_resolution);
  downsample_filter_ = voxelgrid;

  RCLCPP_INFO(this->get_logger(),
              "create registration method for localization");
  registration_ = create_registration();

  // global localization
  RCLCPP_INFO(this->get_logger(),
              "create registration method for fallback during relocalization");
  relocalizing_ = false;
  delta_estimater_.reset(new DeltaEstimater(create_registration()));

  points_topic_ = this->get_parameter("points_topic").as_string();
  imu_topic_    = this->get_parameter("imu_topic").as_string();

  enable_robot_odometry_prediction_ =
      this->get_parameter("enable_robot_odometry_prediction").as_bool();

  // initialize pose estimator
  if (this->get_parameter("specify_init_pose").as_bool()) {
    RCLCPP_INFO(this->get_logger(),
                "initialize pose estimator with specified parameters!!");
    pose_estimator_.reset(new hdl_localization::PoseEstimator(
        registration_,
        Eigen::Vector3f(this->get_parameter("init_pos_x").as_double(),
                        this->get_parameter("init_pos_y").as_double(),
                        this->get_parameter("init_pos_z").as_double()),
        Eigen::Quaternionf(this->get_parameter("init_ori_w").as_double(),
                           this->get_parameter("init_ori_x").as_double(),
                           this->get_parameter("init_ori_y").as_double(),
                           this->get_parameter("init_ori_z").as_double()),
        this->get_parameter("cool_time_duration").as_double()));
  }

  // Log the parameters
  RCLCPP_INFO(this->get_logger(), "--------------------------------");
  RCLCPP_INFO(this->get_logger(), "HDL Localization Node Parameters:");
  RCLCPP_INFO(this->get_logger(), "--------------------------------");
  RCLCPP_INFO(this->get_logger(), "robot_odom_frame_id: %s",
              robot_odom_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "odom_child_frame_id: %s",
              odom_child_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "use_imu: %s", use_imu_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "invert_acc: %s",
              invert_acc_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "invert_gyro: %s",
              invert_gyro_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "points_topic: %s", points_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "reg_method: %s", reg_method_.c_str());
  RCLCPP_INFO(this->get_logger(), "ndt_neighbor_search_method: %s",
              ndt_neighbor_search_method_.c_str());
  RCLCPP_INFO(this->get_logger(), "ndt_neighbor_search_radius: %f",
              ndt_neighbor_search_radius_);
  RCLCPP_INFO(this->get_logger(), "ndt_resolution: %f", ndt_resolution_);
  RCLCPP_INFO(this->get_logger(), "downsample_resolution: %f",
              downsample_resolution);
  RCLCPP_INFO(this->get_logger(), "--------------------------------");
}

void HdlLocalizationNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  std::lock_guard<std::mutex> lock(imu_data_mutex_);
  imu_data_.push_back(imu_msg);
}

void HdlLocalizationNode::points_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
  if (!globalmap_) {
    RCLCPP_ERROR(this->get_logger(), "globalmap has not been received!!");
    return;
  }

  const auto                  &stamp = points_msg->header.stamp;
  pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*points_msg, *pcl_cloud);

  if (pcl_cloud->empty()) {
    RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
    return;
  }

  // transform pointcloud into odom_child_frame_id
  std::string                  tfError;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  if (tf_buffer_.canTransform(odom_child_frame_id_, pcl_cloud->header.frame_id,
                              stamp, tf2::durationFromSec(0.1), &tfError)) {
    if (!pcl_ros::transformPointCloud(odom_child_frame_id_, *pcl_cloud, *cloud,
                                      tf_buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "point cloud cannot be transformed into target frame!!");
      return;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s", tfError.c_str());
    return;
  }

  auto filtered = downsample(cloud);
  last_scan_    = filtered;

  if (relocalizing_) {
    delta_estimater_->add_frame(filtered);
  }

  std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex_);
  if (!pose_estimator_) {
    RCLCPP_ERROR(this->get_logger(), "waiting for initial pose input!!");
    return;
  }
  // Eigen::Matrix4f before = pose_estimator_->matrix();

  // predict
  if (!use_imu_) {
    pose_estimator_->predict(stamp);
  } else {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    auto                        imu_iter = imu_data_.begin();
    for (; imu_iter != imu_data_.end(); imu_iter++) {
      if (rclcpp::Time(stamp).nanoseconds() <
          rclcpp::Time((*imu_iter)->header.stamp).nanoseconds()) {
        break;
      }
      const auto &acc       = (*imu_iter)->linear_acceleration;
      const auto &gyro      = (*imu_iter)->angular_velocity;
      double      acc_sign  = invert_acc_ ? -1.0 : 1.0;
      double      gyro_sign = invert_gyro_ ? -1.0 : 1.0;
      pose_estimator_->predict((*imu_iter)->header.stamp,
                               acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
                               gyro_sign *
                                   Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
    }
    imu_data_.erase(imu_data_.begin(), imu_iter);
  }

  // odometry-based prediction
  rclcpp::Time last_correction_time = pose_estimator_->last_correction_time();
  if (enable_robot_odometry_prediction_ &&
      last_correction_time.nanoseconds() != 0) {
    geometry_msgs::msg::TransformStamped odom_delta;
    if (tf_buffer_.canTransform(
            odom_child_frame_id_, last_correction_time, odom_child_frame_id_,
            stamp, robot_odom_frame_id_, tf2::durationFromSec(0.1))) {
      odom_delta = tf_buffer_.lookupTransform(
          odom_child_frame_id_, last_correction_time, odom_child_frame_id_,
          stamp, robot_odom_frame_id_, tf2::durationFromSec(0));
    } else if (tf_buffer_.canTransform(
                   odom_child_frame_id_, last_correction_time,
                   odom_child_frame_id_, rclcpp::Time(0), robot_odom_frame_id_,
                   tf2::durationFromSec(0))) {
      odom_delta = tf_buffer_.lookupTransform(
          odom_child_frame_id_, last_correction_time, odom_child_frame_id_,
          rclcpp::Time(0), robot_odom_frame_id_, tf2::durationFromSec(0));
    }

    if (odom_delta.header.stamp.nanosec == 0) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "failed to look up transform between "
                             << cloud->header.frame_id << " and "
                             << robot_odom_frame_id_);
    } else {
      Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
      pose_estimator_->predict_odom(delta.cast<float>().matrix());
    }
  }

  // correct
  auto aligned = pose_estimator_->correct(stamp, filtered);

  if (aligned_pub_->get_subscription_count() > 0) {
    aligned->header.frame_id = "map";
    aligned->header.stamp    = cloud->header.stamp;
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*aligned, aligned_msg);
    aligned_pub_->publish(aligned_msg);
  }

  if (status_pub_->get_subscription_count() > 0) {
    publish_scan_matching_status(points_msg->header, aligned);
  }

  publish_odometry(stamp, pose_estimator_->matrix());
}

void HdlLocalizationNode::globalmap_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
  RCLCPP_INFO(this->get_logger(), "globalmap received!");
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*points_msg, *cloud);
  globalmap_ = cloud;

  registration_->setInputTarget(globalmap_);

  if (use_global_localization_) {
    RCLCPP_INFO(this->get_logger(), "set globalmap for global localization!");
    auto request =
        std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
    pcl::toROSMsg(*globalmap_, request->global_map);

    auto response =
        set_global_map_client_->invoke(request, std::chrono::seconds(15));
    if (!response) {
      RCLCPP_ERROR(this->get_logger(), "failed to set global map");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "global map set successfully");
  }
}

bool HdlLocalizationNode::relocalize(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) {
  if (!last_scan_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "no scan has been received");
    return false;
  }

  if (!use_global_localization_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "global localization is disabled");
    return false;
  }

  relocalizing_ = true;
  delta_estimater_->reset();
  pcl::PointCloud<PointT>::ConstPtr scan = last_scan_;

  auto request = std::make_shared<
      hdl_global_localization::srv::QueryGlobalLocalization::Request>();
  pcl::toROSMsg(*scan, request->cloud);
  request->max_num_candidates = 1;
  try {
    RCLCPP_INFO_STREAM(this->get_logger(), "querying global localization");

    auto response = query_global_localization_client_->invoke(
        request, std::chrono::seconds(15));
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "failed to get global localization result");
      return false;
    }
    const auto &result = response->poses[0];

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "--- Global localization result ---");
    RCLCPP_INFO_STREAM(this->get_logger(), "Trans :" << result.position.x << " "
                                                     << result.position.y << " "
                                                     << result.position.z);
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Quat  :" << result.orientation.x << " " << result.orientation.y << " "
                  << result.orientation.z << " " << result.orientation.w);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() =
        Eigen::Quaternionf(result.orientation.w, result.orientation.x,
                           result.orientation.y, result.orientation.z)
            .toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y,
                                         result.position.z);
    pose               = pose * delta_estimater_->estimated_delta();

    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      pose_estimator_.reset(new hdl_localization::PoseEstimator(
          registration_, pose.translation(), Eigen::Quaternionf(pose.linear()),
          this->get_parameter("cool_time_duration").as_double()));
    }

    relocalizing_ = false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "failed to relocalize: %s", e.what());
    relocalizing_ = false;
    return false;
  }
  return true;
}

void HdlLocalizationNode::initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
  RCLCPP_INFO(this->get_logger(), "initial pose received!!");
  relocalizing_ = true;
  delta_estimater_->reset();
  {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    const auto                 &p = pose_msg->pose.pose.position;
    const auto                 &q = pose_msg->pose.pose.orientation;
    pose_estimator_.reset(new hdl_localization::PoseEstimator(
        registration_, Eigen::Vector3f(p.x, p.y, p.z),
        Eigen::Quaternionf(q.w, q.x, q.y, q.z),
        this->get_parameter("cool_time_duration").as_double()));
  }
  relocalizing_ = false;
}

void HdlLocalizationNode::publish_scan_matching_status(
    const std_msgs::msg::Header             &header,
    const pcl::PointCloud<PointT>::ConstPtr &aligned) {
  // publish scan matching status
  hdl_localization::msg::ScanMatchingStatus status;
  status.header         = header;
  status.has_converged  = registration_->hasConverged();
  status.matching_error = 0.0;

  const double max_correspondence_dist =
      this->get_parameter("status_max_correspondence_dist").as_double();
  const double max_valid_point_dist =
      this->get_parameter("status_max_valid_point_dist").as_double();

  int                num_inliers      = 0;
  int                num_valid_points = 0;
  std::vector<int>   k_indices;
  std::vector<float> k_sq_dists;

  for (size_t i = 0; i < aligned->size(); i++) {
    const auto &pt = aligned->at(i);
    if (pt.getVector3fMap().norm() > max_valid_point_dist) {
      continue;
    }
    num_valid_points++;

    registration_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices,
                                                           k_sq_dists);
    if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
      status.matching_error += k_sq_dists[0];
      num_inliers++;
    }
  }

  status.matching_error /= std::max(1, num_inliers);
  status.inlier_fraction =
      static_cast<float>(num_inliers) / std::max(1, num_valid_points);

  // Convert transformation matrix to geometry_msgs::Transform
  Eigen::Matrix4f    final_transform = registration_->getFinalTransformation();
  Eigen::Vector3f    translation     = final_transform.block<3, 1>(0, 3);
  Eigen::Quaternionf rotation(final_transform.block<3, 3>(0, 0));

  geometry_msgs::msg::Transform relative_transform;
  relative_transform.translation.x = translation.x();
  relative_transform.translation.y = translation.y();
  relative_transform.translation.z = translation.z();
  relative_transform.rotation.w    = rotation.w();
  relative_transform.rotation.x    = rotation.x();
  relative_transform.rotation.y    = rotation.y();
  relative_transform.rotation.z    = rotation.z();

  status.relative_pose = relative_transform;

  // Add prediction error information
  if (pose_estimator_->wo_prediction_error()) {
    std_msgs::msg::String label;
    label.data = "without_pred";
    status.prediction_labels.push_back(label);

    Eigen::Matrix4f error_matrix = pose_estimator_->wo_prediction_error().get();
    Eigen::Vector3f error_translation = error_matrix.block<3, 1>(0, 3);
    Eigen::Quaternionf error_rotation(error_matrix.block<3, 3>(0, 0));

    geometry_msgs::msg::Transform error_transform;
    error_transform.translation.x = error_translation.x();
    error_transform.translation.y = error_translation.y();
    error_transform.translation.z = error_translation.z();
    error_transform.rotation.w    = error_rotation.w();
    error_transform.rotation.x    = error_rotation.x();
    error_transform.rotation.y    = error_rotation.y();
    error_transform.rotation.z    = error_rotation.z();

    status.prediction_errors.push_back(error_transform);
  }

  if (pose_estimator_->imu_prediction_error()) {
    std_msgs::msg::String label;
    label.data = use_imu_ ? "imu" : "motion_model";
    status.prediction_labels.push_back(label);

    Eigen::Matrix4f error_matrix =
        pose_estimator_->imu_prediction_error().get();
    Eigen::Vector3f    error_translation = error_matrix.block<3, 1>(0, 3);
    Eigen::Quaternionf error_rotation(error_matrix.block<3, 3>(0, 0));

    geometry_msgs::msg::Transform error_transform;
    error_transform.translation.x = error_translation.x();
    error_transform.translation.y = error_translation.y();
    error_transform.translation.z = error_translation.z();
    error_transform.rotation.w    = error_rotation.w();
    error_transform.rotation.x    = error_rotation.x();
    error_transform.rotation.y    = error_rotation.y();
    error_transform.rotation.z    = error_rotation.z();

    status.prediction_errors.push_back(error_transform);
  }

  if (pose_estimator_->odom_prediction_error()) {
    std_msgs::msg::String label;
    label.data = "odom";
    status.prediction_labels.push_back(label);

    Eigen::Matrix4f error_matrix =
        pose_estimator_->odom_prediction_error().get();
    Eigen::Vector3f    error_translation = error_matrix.block<3, 1>(0, 3);
    Eigen::Quaternionf error_rotation(error_matrix.block<3, 3>(0, 0));

    geometry_msgs::msg::Transform error_transform;
    error_transform.translation.x = error_translation.x();
    error_transform.translation.y = error_translation.y();
    error_transform.translation.z = error_translation.z();
    error_transform.rotation.w    = error_rotation.w();
    error_transform.rotation.x    = error_rotation.x();
    error_transform.rotation.y    = error_rotation.y();
    error_transform.rotation.z    = error_rotation.z();

    status.prediction_errors.push_back(error_transform);
  }

  status_pub_->publish(status);
}

void HdlLocalizationNode::publish_odometry(const rclcpp::Time    &stamp,
                                           const Eigen::Matrix4f &pose) {
  // broadcast the transform over tf
  std::string tfError;
  if (tf_buffer_.canTransform(robot_odom_frame_id_, odom_child_frame_id_,
                              rclcpp::Time(0), tf2::durationFromSec(0.1),
                              &tfError)) {
    // Complex transform logic for robot odometry frame
    Eigen::Isometry3d map_wrt_frame =
        Eigen::Isometry3d(pose.inverse().cast<double>());

    geometry_msgs::msg::TransformStamped map_wrt_frame_msg;
    map_wrt_frame_msg.header.stamp    = stamp;
    map_wrt_frame_msg.header.frame_id = odom_child_frame_id_;
    map_wrt_frame_msg.child_frame_id  = "map";
    map_wrt_frame_msg.transform =
        tf2::eigenToTransform(map_wrt_frame).transform;

    try {
      geometry_msgs::msg::TransformStamped frame_wrt_odom =
          tf_buffer_.lookupTransform(robot_odom_frame_id_, odom_child_frame_id_,
                                     rclcpp::Time(0),
                                     tf2::durationFromSec(0.1));

      geometry_msgs::msg::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame_msg, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.transform       = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp    = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id  = robot_odom_frame_id_;

      tf_broadcaster_.sendTransform(odom_trans);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      // Fallback to simple transform
      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.header.stamp    = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id  = odom_child_frame_id_;
      odom_trans.transform =
          tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()))
              .transform;
      tf_broadcaster_.sendTransform(odom_trans);
    }
  } else {
    // Simple transform when no robot odometry frame
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp    = stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id  = odom_child_frame_id_;
    odom_trans.transform =
        tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>())).transform;
    tf_broadcaster_.sendTransform(odom_trans);
  }

  // publish odometry
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id  = odom_child_frame_id_;

  odom.pose.pose.position.x = pose(0, 3);
  odom.pose.pose.position.y = pose(1, 3);
  odom.pose.pose.position.z = pose(2, 3);

  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.linear.y  = 0.0;
  odom.twist.twist.linear.z  = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  pose_pub_->publish(odom);
}

pcl::PointCloud<HdlLocalizationNode::PointT>::ConstPtr
HdlLocalizationNode::downsample(
    const pcl::PointCloud<HdlLocalizationNode::PointT>::ConstPtr &cloud) const {
  if (!downsample_filter_) {
    return cloud;
  }

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  downsample_filter_->setInputCloud(cloud);
  downsample_filter_->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

} // namespace hdl_localization

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationNode)