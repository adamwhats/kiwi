#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/Geometry>
#include <chrono>

#include <rcutils/logging.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace kiwi_perception
{

struct ClusterInfo {
  Eigen::Vector3f obb_position;
  Eigen::Quaternionf obb_orientation;
  Eigen::Vector3f obb_dimensions;
  float min_dimension;
  float vertical_extent;
  bool is_graspable;
  int cluster_id;
};

class GraspPlanner : public rclcpp::Node
{
public:
  GraspPlanner(const rclcpp::NodeOptions& options) : Node("grasp_planner", options)
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

    process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud", 10, std::bind(&GraspPlanner::pointcloud_callback, this, std::placeholders::_1));

    // Publishers
    processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/processed_points", 10);
    cluster_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/cluster_markers", 10);

    RCLCPP_INFO(this->get_logger(), "GraspPlanner initialized with clip_distance=%.2f, ground_height=%.2f, voxel=%.3f",
                clip_distance_, ground_height_, voxel_leaf_size_);
  }

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;

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

    auto markers = createClusterMarkers(clusters, msg->header.stamp);
    cluster_marker_pub_->publish(markers);

    auto t_total_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "TIMING: total=%.1fms | pts: %zu->%zu->%zu | convert=%.1f clip=%.1f voxel=%.1f xform=%.1f ground=%.1f kdtree=%.1f cluster=%.1f obb=%.1f | %zu clusters",
      total_ms, input_points, after_voxel, after_ground, convert_ms, clip_ms, voxel_ms, transform_ms, ground_ms, kdtree_ms, cluster_ms, obb_ms, clusters.size());
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cluster_cloud->reserve(indices.indices.size());

      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();

      for (const auto& idx : indices.indices) {
        const auto& pt = (*cloud)[idx];
        cluster_cloud->push_back(pt);
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
      }

      cluster_cloud->width = cluster_cloud->size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

      ClusterInfo info;
      info.cluster_id = cluster_id++;
      info.vertical_extent = max_z - min_z;
      info.is_graspable = false;

      if (cluster_cloud->size() >= 3) {
        auto t_obb_start = std::chrono::high_resolution_clock::now();

        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();

        pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        info.obb_orientation = Eigen::Quaternionf(rotational_matrix_OBB);

        Eigen::Vector3f min_pt(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f max_pt(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        info.obb_dimensions = (max_pt - min_pt).cwiseAbs();

        info.obb_position = Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);
        info.min_dimension = info.obb_dimensions.minCoeff();

        auto t_obb_end = std::chrono::high_resolution_clock::now();
        obb_ms += std::chrono::duration<double, std::milli>(t_obb_end - t_obb_start).count();

        clusters.push_back(info);
      }
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

  visualization_msgs::msg::MarkerArray createClusterMarkers(
      const std::vector<ClusterInfo>& clusters,
      const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(clusters.size() + 1);

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
    }

    return marker_array;
  }
};

} // namespace kiwi_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_perception::GraspPlanner)
