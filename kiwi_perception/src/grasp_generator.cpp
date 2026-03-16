#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <mutex>
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kiwi_interfaces/action/select_grasp_target.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace kiwi_perception {

using SelectGraspTarget = kiwi_interfaces::action::SelectGraspTarget;
using GoalHandle = rclcpp_action::ServerGoalHandle<SelectGraspTarget>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double quat_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

// Return rotation matrix column for unit approach vector given yaw
static std::pair<double, double> yaw_to_unit(double yaw) {
  return {std::cos(yaw), std::sin(yaw)};
}

// Offset pose backwards along its local Z axis (approach direction)
static geometry_msgs::msg::PoseStamped offset_along_approach(const geometry_msgs::msg::PoseStamped& pose, double dist) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(pose.pose.orientation, tf_q);
  const tf2::Vector3 approach = tf2::Matrix3x3(tf_q).getColumn(2);

  geometry_msgs::msg::PoseStamped out = pose;
  out.pose.position.x -= dist * approach.x();
  out.pose.position.y -= dist * approach.y();
  out.pose.position.z -= dist * approach.z();
  return out;
}

// Build a PoseStamped from position + yaw (z-only rotation)
static geometry_msgs::msg::PoseStamped make_pose(const std::string& frame, double x, double y, double z, double yaw) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

// Build grasp TCP pose: Z=approach, Y=fingers (short axis), X=Y×Z (X points down)
static geometry_msgs::msg::PoseStamped make_tcp_pose(const std::string& frame, double x, double y, double z,
                                                     double approach_yaw, double arm_base_x, double arm_base_y) {
  // Approach along long OBB axis facing the arm base
  double finger_yaw = approach_yaw + M_PI_2;

  double arm_dir = std::atan2(y - arm_base_y, x - arm_base_x);
  double diff = approach_yaw - arm_dir;
  while (diff > M_PI)
    diff -= 2.0 * M_PI;
  while (diff < -M_PI)
    diff += 2.0 * M_PI;
  double actual_approach = (std::abs(diff) > M_PI_2) ? approach_yaw + M_PI : approach_yaw;

  double za[3] = {std::cos(actual_approach), std::sin(actual_approach), 0.0};
  double ya[3] = {std::cos(finger_yaw), std::sin(finger_yaw), 0.0};

  // X = Y × Z
  double xa[3] = {ya[1] * za[2] - ya[2] * za[1], ya[2] * za[0] - ya[0] * za[2], ya[0] * za[1] - ya[1] * za[0]};
  if (xa[2] > 0.0) {
    xa[0] = -xa[0];
    xa[1] = -xa[1];
    xa[2] = -xa[2];
    ya[0] = -ya[0];
    ya[1] = -ya[1];
    ya[2] = -ya[2];
  }

  // Get rotation from XYZ axis directions
  tf2::Matrix3x3 rot(xa[0], ya[0], za[0], xa[1], ya[1], za[1], xa[2], ya[2], za[2]);
  tf2::Quaternion q;
  rot.getRotation(q);

  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation = tf2::toMsg(q);
  return p;
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

struct GraspCandidate {
  double robot_x, robot_y, robot_yaw;
  double tcp_x, tcp_y, tcp_z;
  double tcp_yaw;  // OBB long-axis yaw (used for finger orientation)
  double arm_base_x_world, arm_base_y_world;
  int costmap_cost;
  geometry_msgs::msg::PoseStamped obb_pose_map;
  geometry_msgs::msg::Vector3 obb_scale;
};

class GraspGenerator : public rclcpp::Node {
 public:
  explicit GraspGenerator(const rclcpp::NodeOptions& options) : Node("grasp_generator", options) {
    declare_parameter("arm_base_x", 0.07425);
    declare_parameter("arm_base_y", 0.076);
    declare_parameter("arm_base_z", 0.05195);
    declare_parameter("arm_reach_min", 0.08);
    declare_parameter("arm_reach_max", 0.30);
    declare_parameter("dz_min", -0.25);
    declare_parameter("dz_max", 0.10);
    declare_parameter("n_elevation_samples", 5);
    declare_parameter("costmap_lethal_threshold", 253);
    declare_parameter("base_frame", std::string("base_link"));
    declare_parameter("map_frame", std::string("map"));

    arm_base_x_ = get_parameter("arm_base_x").as_double();
    arm_base_y_ = get_parameter("arm_base_y").as_double();
    arm_base_z_ = get_parameter("arm_base_z").as_double();
    arm_reach_min_ = get_parameter("arm_reach_min").as_double();
    arm_reach_max_ = get_parameter("arm_reach_max").as_double();
    dz_min_ = get_parameter("dz_min").as_double();
    dz_max_ = get_parameter("dz_max").as_double();
    n_samples_ = get_parameter("n_elevation_samples").as_int();
    lethal_ = get_parameter("costmap_lethal_threshold").as_int();
    base_frame_ = get_parameter("base_frame").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    markers_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "~/cluster_markers", 10, [this](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          latest_markers_ = std::move(msg);
        });

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(1), [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          costmap_ = std::move(msg);
        });

    action_server_ = rclcpp_action::create_server<SelectGraspTarget>(
        this, "~/select_target",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const SelectGraspTarget::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<GoalHandle> gh) { execute(gh); });

    grasp_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/selected", 10);

    RCLCPP_INFO(get_logger(), "GraspGenerator ready");
  }

 private:
  // -------------------------------------------------------------------------
  // Action execution
  // -------------------------------------------------------------------------

  void execute(std::shared_ptr<GoalHandle> goal_handle) {
    const float standoff = goal_handle->get_goal()->standoff_distance;
    auto result = std::make_shared<SelectGraspTarget::Result>();

    visualization_msgs::msg::MarkerArray::SharedPtr markers;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      markers = latest_markers_;
      costmap = costmap_;
    }

    if (!markers || markers->markers.empty()) {
      result->success = false;
      result->message = "No object detections";
      goal_handle->abort(result);
      return;
    }

    // Transform base_link → map for OBB centroids
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      result->success = false;
      result->message = std::string("TF error: ") + e.what();
      goal_handle->abort(result);
      return;
    }

    std::vector<GraspCandidate> candidates;
    int n_graspable = 0;

    for (const auto& m : markers->markers) {
      if (m.ns != "graspable")
        continue;
      if (m.action != visualization_msgs::msg::Marker::ADD)
        continue;

      ++n_graspable;

      // Transform OBB centroid to map frame
      geometry_msgs::msg::PoseStamped centroid_base, centroid_map;
      centroid_base.header.frame_id = base_frame_;
      centroid_base.pose = m.pose;
      tf2::doTransform(centroid_base, centroid_map, tf_stamped);

      const double ox = centroid_map.pose.position.x;
      const double oy = centroid_map.pose.position.y;
      const double oz = centroid_map.pose.position.z;
      const double obb_yaw = quat_to_yaw(centroid_map.pose.orientation);

      // Try both long-axis approach azimuths
      for (int dir = 0; dir < 2; ++dir) {
        const double approach_yaw = obb_yaw + dir * M_PI;

        auto [cos_a, sin_a] = yaw_to_unit(approach_yaw);

        // Sample elevation angles on the approach semicircle
        for (int k = 0; k < n_samples_; ++k) {
          const double theta = -M_PI_2 + M_PI * k / std::max(n_samples_ - 1, 1);
          const double r = arm_reach_max_ * std::cos(theta);
          const double dz = arm_reach_max_ * std::sin(theta);

          if (r < arm_reach_min_ || r > arm_reach_max_)
            continue;
          if (dz < dz_min_ || dz > dz_max_)
            continue;

          // arm_base must be r metres behind the TCP along approach, in map frame
          const double arm_bx = ox - r * cos_a;
          const double arm_by = oy - r * sin_a;

          // robot position: subtract arm_base offset rotated by approach_yaw
          const double rx = arm_bx - (arm_base_x_ * cos_a - arm_base_y_ * sin_a);
          const double ry = arm_by - (arm_base_x_ * sin_a + arm_base_y_ * cos_a);

          const int cost = costmap_cost_at(costmap, rx, ry);
          if (cost < 0 || cost >= lethal_)
            continue;

          candidates.push_back(
              {rx, ry, approach_yaw, ox, oy, oz, obb_yaw, arm_bx, arm_by, cost, centroid_map, m.scale});
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Grasp search: %d objects, %d positions sampled, %zu valid", n_graspable,
                n_graspable * 2 * n_samples_, candidates.size());

    if (candidates.empty()) {
      result->success = false;
      result->message = "No feasible grasp candidates";
      goal_handle->abort(result);
      return;
    }

    // Pick lowest costmap cost
    const auto& best = *std::min_element(
        candidates.begin(), candidates.end(),
        [](const GraspCandidate& a, const GraspCandidate& b) { return a.costmap_cost < b.costmap_cost; });

    // base_pose in map frame
    result->base_pose = make_pose(map_frame_, best.robot_x, best.robot_y, 0.0, best.robot_yaw);
    result->base_pose.header.stamp = now();

    // grasp_pose: TCP expressed in base_link frame as if robot were at base_pose
    // TCP map position is the object centroid
    geometry_msgs::msg::PoseStamped tcp_map = make_tcp_pose(map_frame_, best.tcp_x, best.tcp_y, best.tcp_z,
                                                            best.tcp_yaw, best.arm_base_x_world, best.arm_base_y_world);
    tcp_map.header.stamp = now();

    // Express TCP relative to robot at base_pose (= base_link when robot arrives)
    tf2::Transform base_tf, tcp_tf, rel_tf;
    tf2::fromMsg(result->base_pose.pose, base_tf);
    tf2::fromMsg(tcp_map.pose, tcp_tf);
    rel_tf = base_tf.inverse() * tcp_tf;

    result->grasp_pose.header.frame_id = base_frame_;
    result->grasp_pose.header.stamp = now();
    tf2::toMsg(rel_tf, result->grasp_pose.pose);

    result->pre_pick_pose = offset_along_approach(result->grasp_pose, static_cast<double>(standoff));
    result->success = true;
    result->message = "ok";
    goal_handle->succeed(result);
    publish_grasp_marker(best);
  }

  // -------------------------------------------------------------------------
  // Visualization
  // -------------------------------------------------------------------------

  void publish_grasp_marker(const GraspCandidate& best) {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = map_frame_;
    del.ns = "selected_grasp";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    visualization_msgs::msg::Marker m;
    m.header = best.obb_pose_map.header;
    m.header.stamp = now();
    m.ns = "selected_grasp";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = best.obb_pose_map.pose;
    m.scale.x = best.obb_scale.x * 1.05;
    m.scale.y = best.obb_scale.y * 1.05;
    m.scale.z = best.obb_scale.z * 1.05;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.7;
    m.lifetime = rclcpp::Duration::from_seconds(30.0);
    ma.markers.push_back(m);

    grasp_viz_pub_->publish(ma);
  }

  // -------------------------------------------------------------------------
  // Costmap lookup
  // -------------------------------------------------------------------------

  int costmap_cost_at(const nav_msgs::msg::OccupancyGrid::SharedPtr& map, double wx, double wy) const {
    if (!map)
      return -1;
    const auto& info = map->info;
    const double mx = (wx - info.origin.position.x) / info.resolution;
    const double my = (wy - info.origin.position.y) / info.resolution;
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(info.width) || iy >= static_cast<int>(info.height))
      return -1;
    return static_cast<int>(map->data[iy * info.width + ix]);
  }

  // -------------------------------------------------------------------------
  // Members
  // -------------------------------------------------------------------------

  double arm_base_x_, arm_base_y_, arm_base_z_;
  double arm_reach_min_, arm_reach_max_;
  double dz_min_, dz_max_;
  int n_samples_, lethal_;
  std::string base_frame_, map_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp_action::Server<SelectGraspTarget>::SharedPtr action_server_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasp_viz_pub_;

  std::mutex mutex_;
  visualization_msgs::msg::MarkerArray::SharedPtr latest_markers_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
};

}  // namespace kiwi_perception

RCLCPP_COMPONENTS_REGISTER_NODE(kiwi_perception::GraspGenerator)
