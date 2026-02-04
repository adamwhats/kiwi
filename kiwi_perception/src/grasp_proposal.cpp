#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>

#include <rcutils/logging.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace kiwi_perception
{

struct ClusterInfo {
  Eigen::Vector3f obb_position;       // center in base_link
  Eigen::Quaternionf obb_orientation; // yaw-only rotation around Z
  Eigen::Vector3f obb_dimensions;     // (extent_pca1, extent_pca2, vertical)
  float yaw;                          // PCA yaw angle in base_link
  float min_dimension;                // smallest horizontal OBB dimension
  float vertical_extent;              // max_z - min_z
  bool is_graspable;
  int cluster_id;
};

class GraspProposal : public rclcpp::Node
{
public:
  GraspProposal(const rclcpp::NodeOptions& options) : Node("grasp_proposal", options)
  {
    // Pointcloud filter parameters
    this->declare_parameter("clip_distance", 0.5);
    this->declare_parameter("ground_height", 0.01);
    this->declare_parameter("base_frame", "base_link");

    // Clustering parameters
    this->declare_parameter("cluster_tolerance", 0.02);
    this->declare_parameter("min_cluster_size", 100);
    this->declare_parameter("max_cluster_size", 800);

    // Graspability filter parameters
    this->declare_parameter("gripper_width", 0.08);
    this->declare_parameter("gripper_width_factor", 0.9);
    this->declare_parameter("max_cluster_height", 0.12);
    this->declare_parameter("min_cluster_height", 0.01);
    this->declare_parameter("max_obb_dimension", 0.15);

    // Arm base position in base_link (from URDF link0 origin)
    this->declare_parameter("arm_base_x", 0.07425);
    this->declare_parameter("arm_base_y", 0.076);

    // Performance parameters
    this->declare_parameter("process_every_n_frames", 1);
    this->declare_parameter("voxel_leaf_size", 0.005);

    // Log level for composable nodes (--ros-args --log-level doesn't propagate)
    this->declare_parameter("log_level", "info");
    auto log_level = this->get_parameter("log_level").as_string();
    int severity = RCUTILS_LOG_SEVERITY_INFO;
    if (log_level == "debug") severity = RCUTILS_LOG_SEVERITY_DEBUG;
    else if (log_level == "warn") severity = RCUTILS_LOG_SEVERITY_WARN;
    else if (log_level == "error") severity = RCUTILS_LOG_SEVERITY_ERROR;
    else if (log_level == "fatal") severity = RCUTILS_LOG_SEVERITY_FATAL;
    (void)rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);

    // Get parameters
    clip_distance_ = this->get_parameter("clip_distance").as_double();
    ground_height_ = this->get_parameter("ground_height").as_double();
    base_frame_ = this->get_parameter("base_frame").as_string();

    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

    gripper_width_ = this->get_parameter("gripper_width").as_double();
    gripper_width_factor_ = this->get_parameter("gripper_width_factor").as_double();
    max_cluster_height_ = this->get_parameter("max_cluster_height").as_double();
    min_cluster_height_ = this->get_parameter("min_cluster_height").as_double();
    max_obb_dimension_ = this->get_parameter("max_obb_dimension").as_double();

    arm_base_x_ = this->get_parameter("arm_base_x").as_double();
    arm_base_y_ = this->get_parameter("arm_base_y").as_double();

    process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud", 10, std::bind(&GraspProposal::pointcloud_callback, this, std::placeholders::_1));

    // Publishers
    processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/processed_points", 10);
    cluster_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/cluster_markers", 10);
    grasp_proposals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/grasp_proposals", 10);

    RCLCPP_INFO(this->get_logger(), "GraspProposal initialized with clip_distance=%.2f, ground_height=%.2f, voxel=%.3f",
                clip_distance_, ground_height_, voxel_leaf_size_);
  }

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr grasp_proposals_pub_;

  // TF
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters - filtering
  double clip_distance_;
  double ground_height_;

  // Parameters - clustering
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  // Parameters - graspability
  double gripper_width_;
  double gripper_width_factor_;
  double max_cluster_height_;
  double min_cluster_height_;
  double max_obb_dimension_;

  // Arm base position
  double arm_base_x_;
  double arm_base_y_;

  // Performance
  int process_every_n_frames_;
  int frame_counter_ = 0;
  double voxel_leaf_size_;

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (++frame_counter_ < process_every_n_frames_) {
      return;
    }
    frame_counter_ = 0;

    auto t_total_start = std::chrono::high_resolution_clock::now();

    // Convert ROS PointCloud2 to PCL
    auto t0 = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    auto t1 = std::chrono::high_resolution_clock::now();
    double convert_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    size_t input_points = cloud->size();

    // Clip in camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clipped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, clip_distance_);
    pass_z.filter(*cloud_clipped);
    auto t2 = std::chrono::high_resolution_clock::now();
    double clip_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

    // Voxel downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled = cloud_clipped;
    double voxel_ms = 0.0;
    if (voxel_leaf_size_ > 0.0) {
      cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud_clipped);
      voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel.filter(*cloud_downsampled);
      auto t2b = std::chrono::high_resolution_clock::now();
      voxel_ms = std::chrono::duration<double, std::milli>(t2b - t2).count();
      t2 = t2b;
    }

    size_t after_voxel = cloud_downsampled->size();

    // Transform to base frame
    sensor_msgs::msg::PointCloud2 cloud_clipped_msg;
    pcl::toROSMsg(*cloud_downsampled, cloud_clipped_msg);
    cloud_clipped_msg.header = msg->header;

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(base_frame_, cloud_clipped_msg, cloud_transformed, *tf_buffer_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Could not transform pointcloud to %s", base_frame_.c_str());
      return;
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    double transform_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Convert back to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_transformed, *cloud_base);

    // Ground removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_ground;
    pass_ground.setInputCloud(cloud_base);
    pass_ground.setFilterFieldName("z");
    pass_ground.setFilterLimits(ground_height_, std::numeric_limits<float>::max());
    pass_ground.filter(*cloud_no_ground);
    auto t4 = std::chrono::high_resolution_clock::now();
    double ground_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();

    size_t after_ground = cloud_no_ground->size();

    // Publish processed cloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_no_ground, output_msg);
    output_msg.header.frame_id = base_frame_;
    output_msg.header.stamp = msg->header.stamp;
    processed_cloud_pub_->publish(output_msg);

    // Clustering and OBB
    double kdtree_ms, cluster_ms, obb_ms;
    auto clusters = extractAndProcessClusters(cloud_no_ground, kdtree_ms, cluster_ms, obb_ms);

    if (clusters.empty()) {
      auto empty_markers = createClusterMarkers({}, msg->header.stamp);
      cluster_marker_pub_->publish(empty_markers);

      auto t_total_end = std::chrono::high_resolution_clock::now();
      double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "TIMING: total=%.1fms | pts: %zu->%zu->%zu | convert=%.1f clip=%.1f voxel=%.1f xform=%.1f ground=%.1f | NO CLUSTERS",
        total_ms, input_points, after_voxel, after_ground, convert_ms, clip_ms, voxel_ms, transform_ms, ground_ms);
      return;
    }

    filterGraspableClusters(clusters);

    // Publish grasp proposals for all graspable clusters
    geometry_msgs::msg::PoseArray proposals;
    proposals.header.frame_id = base_frame_;
    proposals.header.stamp = msg->header.stamp;
    for (const auto& cluster : clusters) {
      if (!cluster.is_graspable) continue;
      proposals.poses.push_back(computeGraspPose(cluster));
    }
    grasp_proposals_pub_->publish(proposals);

    auto markers = createClusterMarkers(clusters, msg->header.stamp);
    cluster_marker_pub_->publish(markers);

    auto t_total_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "TIMING: total=%.1fms | pts: %zu->%zu->%zu | convert=%.1f clip=%.1f voxel=%.1f xform=%.1f ground=%.1f kdtree=%.1f cluster=%.1f obb=%.1f | %zu clusters, %zu graspable",
      total_ms, input_points, after_voxel, after_ground, convert_ms, clip_ms, voxel_ms, transform_ms, ground_ms, kdtree_ms, cluster_ms, obb_ms, clusters.size(), proposals.poses.size());
  }

  std::vector<ClusterInfo> extractAndProcessClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                      double& kdtree_ms, double& cluster_ms, double& obb_ms)
  {
    std::vector<ClusterInfo> clusters;
    kdtree_ms = cluster_ms = obb_ms = 0.0;

    if (cloud->empty()) {
      return clusters;
    }

    auto t0 = std::chrono::high_resolution_clock::now();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    auto t1 = std::chrono::high_resolution_clock::now();
    kdtree_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    auto t2 = std::chrono::high_resolution_clock::now();
    cluster_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

    clusters.reserve(cluster_indices.size());

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
      if (indices.indices.size() < 3) {
        continue;
      }

      auto t_obb_start = std::chrono::high_resolution_clock::now();

      // Accumulate XY stats and z bounds in a single pass
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
      double sum_x = 0.0, sum_y = 0.0;

      for (const auto& idx : indices.indices) {
        const auto& pt = (*cloud)[idx];
        sum_x += pt.x;
        sum_y += pt.y;
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
      }

      const double n = static_cast<double>(indices.indices.size());
      const float cx = static_cast<float>(sum_x / n);
      const float cy = static_cast<float>(sum_y / n);

      // 2D covariance matrix for XY
      double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
      for (const auto& idx : indices.indices) {
        const auto& pt = (*cloud)[idx];
        double dx = pt.x - cx;
        double dy = pt.y - cy;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
      }
      cov_xx /= n;
      cov_xy /= n;
      cov_yy /= n;

      // Eigendecomposition of 2x2 symmetric matrix
      // Eigenvalues of [[a,b],[b,c]]: lambda = (a+c)/2 ± sqrt(((a-c)/2)^2 + b^2)
      double trace = cov_xx + cov_yy;
      double det = cov_xx * cov_yy - cov_xy * cov_xy;
      double disc = std::sqrt(std::max(0.0, trace * trace / 4.0 - det));

      // Principal eigenvector
      Eigen::Vector2f axis1;
      if (std::abs(cov_xy) > 1e-8) {
        double lambda1 = trace / 2.0 + disc;
        axis1 = Eigen::Vector2f(static_cast<float>(cov_xy),
                                static_cast<float>(lambda1 - cov_xx)).normalized();
      } else {
        // Already axis-aligned, pick the axis with larger variance
        axis1 = (cov_xx >= cov_yy) ? Eigen::Vector2f(1, 0) : Eigen::Vector2f(0, 1);
      }
      Eigen::Vector2f axis2(-axis1.y(), axis1.x()); // perpendicular

      // Project points onto PCA axes to get extents
      float min_a1 = std::numeric_limits<float>::max(), max_a1 = std::numeric_limits<float>::lowest();
      float min_a2 = std::numeric_limits<float>::max(), max_a2 = std::numeric_limits<float>::lowest();
      for (const auto& idx : indices.indices) {
        const auto& pt = (*cloud)[idx];
        float dx = pt.x - cx;
        float dy = pt.y - cy;
        float proj1 = dx * axis1.x() + dy * axis1.y();
        float proj2 = dx * axis2.x() + dy * axis2.y();
        min_a1 = std::min(min_a1, proj1);
        max_a1 = std::max(max_a1, proj1);
        min_a2 = std::min(min_a2, proj2);
        max_a2 = std::max(max_a2, proj2);
      }

      float extent1 = max_a1 - min_a1;
      float extent2 = max_a2 - min_a2;
      float center_offset1 = (max_a1 + min_a1) / 2.0f;
      float center_offset2 = (max_a2 + min_a2) / 2.0f;

      ClusterInfo info;
      info.cluster_id = cluster_id++;
      info.vertical_extent = max_z - min_z;
      info.is_graspable = false;

      // OBB center: centroid + offset along PCA axes
      info.obb_position = Eigen::Vector3f(
        cx + center_offset1 * axis1.x() + center_offset2 * axis2.x(),
        cy + center_offset1 * axis1.y() + center_offset2 * axis2.y(),
        (min_z + max_z) / 2.0f);

      info.obb_dimensions = Eigen::Vector3f(extent1, extent2, max_z - min_z);
      info.yaw = std::atan2(axis1.y(), axis1.x());

      // Quaternion from yaw-only rotation around Z
      info.obb_orientation = Eigen::Quaternionf(Eigen::AngleAxisf(info.yaw, Eigen::Vector3f::UnitZ()));

      // Smallest horizontal dimension (what the gripper must straddle)
      info.min_dimension = std::min(extent1, extent2);

      auto t_obb_end = std::chrono::high_resolution_clock::now();
      obb_ms += std::chrono::duration<double, std::milli>(t_obb_end - t_obb_start).count();

      clusters.push_back(info);
    }

    return clusters;
  }

  void filterGraspableClusters(std::vector<ClusterInfo>& clusters)
  {
    const double max_grasp_width = gripper_width_ * gripper_width_factor_;

    for (auto& c : clusters) {
      c.is_graspable = true;

      // Reject clusters not resting on the ground plane
      if (c.obb_position.z() > ground_height_ + max_cluster_height_) {
        c.is_graspable = false;
        continue;
      }

      // Reject flat clusters
      if (c.vertical_extent < min_cluster_height_) {
        c.is_graspable = false;
        continue;
      }

      // Reject clusters wider than the gripper width
      if (c.min_dimension > max_grasp_width) {
        c.is_graspable = false;
        continue;
      }

      // Reject clusters which are otherwise too large
      if (c.obb_dimensions.maxCoeff() > max_obb_dimension_) {
        c.is_graspable = false;
        continue;
      }
    }
  }

  geometry_msgs::msg::Pose computeGraspPose(const ClusterInfo& cluster)
  {
    // OBB has two horizontal axes:
    //   axis1 at angle yaw with extent obb_dimensions.x()
    //   axis2 at angle yaw+π/2 with extent obb_dimensions.y()
    // Y (fingers) should straddle the smaller extent
    // Z (approach) should be along the perpendicular OBB axis
    float finger_angle, approach_angle;
    if (cluster.obb_dimensions.x() <= cluster.obb_dimensions.y()) {
      finger_angle = cluster.yaw;
      approach_angle = cluster.yaw + static_cast<float>(M_PI_2);
    } else {
      finger_angle = cluster.yaw + static_cast<float>(M_PI_2);
      approach_angle = cluster.yaw;
    }

    // Pick the approach direction along the OBB axis that faces the arm
    float dx = cluster.obb_position.x() - static_cast<float>(arm_base_x_);
    float dy = cluster.obb_position.y() - static_cast<float>(arm_base_y_);
    float arm_dir = std::atan2(dy, dx);

    float diff = approach_angle - arm_dir;
    while (diff > M_PI) diff -= 2.0f * static_cast<float>(M_PI);
    while (diff < -M_PI) diff += 2.0f * static_cast<float>(M_PI);
    if (std::abs(diff) > M_PI_2) {
      approach_angle += static_cast<float>(M_PI);
    }

    // Construct TCP frame aligned with OBB
    //   Z = approach (along OBB larger axis, toward object from arm side)
    //   Y = finger opening (along OBB smaller axis)
    //   X = Y × Z (vertical for horizontal approach)
    Eigen::Vector3f z_axis(std::cos(approach_angle), std::sin(approach_angle), 0.0f);
    Eigen::Vector3f y_axis(std::cos(finger_angle), std::sin(finger_angle), 0.0f);
    Eigen::Vector3f x_axis = y_axis.cross(z_axis);

    // Ensure right-handed with X pointing down (consistent with arm plane)
    if (x_axis.z() > 0) {
      y_axis = -y_axis;
      x_axis = -x_axis;
    }

    Eigen::Matrix3f rot;
    rot.col(0) = x_axis;
    rot.col(1) = y_axis;
    rot.col(2) = z_axis;
    Eigen::Quaternionf quat(rot);

    geometry_msgs::msg::Pose pose;
    pose.position.x = cluster.obb_position.x();
    pose.position.y = cluster.obb_position.y();
    pose.position.z = cluster.obb_position.z();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
  }

  visualization_msgs::msg::MarkerArray createClusterMarkers(
      const std::vector<ClusterInfo>& clusters,
      const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(clusters.size() * 2 + 1);

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = base_frame_;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "cluster_obb";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto& cluster : clusters) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = base_frame_;
      marker.header.stamp = stamp;
      marker.ns = "cluster_obb";
      marker.id = cluster.cluster_id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = cluster.obb_position.x();
      marker.pose.position.y = cluster.obb_position.y();
      marker.pose.position.z = cluster.obb_position.z();

      marker.pose.orientation.x = cluster.obb_orientation.x();
      marker.pose.orientation.y = cluster.obb_orientation.y();
      marker.pose.orientation.z = cluster.obb_orientation.z();
      marker.pose.orientation.w = cluster.obb_orientation.w();

      marker.scale.x = cluster.obb_dimensions.x();
      marker.scale.y = cluster.obb_dimensions.y();
      marker.scale.z = cluster.obb_dimensions.z();

      if (cluster.is_graspable) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.6;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
      }

      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(marker);

      // Arrow marker for each graspable cluster's approach direction
      if (cluster.is_graspable) {
        auto grasp_pose = computeGraspPose(cluster);

        // Extract approach direction from the grasp pose Z axis
        Eigen::Quaternionf q(grasp_pose.orientation.w, grasp_pose.orientation.x,
                             grasp_pose.orientation.y, grasp_pose.orientation.z);
        Eigen::Vector3f approach = q * Eigen::Vector3f::UnitZ();

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = base_frame_;
        arrow.header.stamp = stamp;
        arrow.ns = "grasp_approach";
        arrow.id = cluster.cluster_id;
        arrow.type = visualization_msgs::msg::Marker::POSE;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.points.resize(2);
        arrow.points[0].x = cluster.obb_position.x() - 0.08f * approach.x();
        arrow.points[0].y = cluster.obb_position.y() - 0.08f * approach.y();
        arrow.points[0].z = cluster.obb_position.z();
        arrow.points[1].x = cluster.obb_position.x();
        arrow.points[1].y = cluster.obb_position.y();
        arrow.points[1].z = cluster.obb_position.z();
        arrow.scale.x = 0.008; // shaft diameter
        arrow.scale.y = 0.015; // head diameter
        arrow.scale.z = 0.02;  // head length
        arrow.color.r = 0.0;
        arrow.color.g = 0.5;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(arrow);
      }
    }

    return marker_array;
  }
};

} // namespace kiwi_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_perception::GraspProposal)
