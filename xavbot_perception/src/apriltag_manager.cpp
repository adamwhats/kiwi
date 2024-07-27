#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "xavbot_interfaces/msg/april_tag_detection_stamped.hpp"
#include "xavbot_interfaces/action/query_april_tag_log.hpp"


using namespace std::placeholders;

namespace xavbot_perception
{
class AprilTagManager : public rclcpp::Node
{
  public:
    using QueryAprilTagLog = xavbot_interfaces::action::QueryAprilTagLog;
    using GoalHandleQueryAprilTagLog = rclcpp_action::ServerGoalHandle<QueryAprilTagLog>;

    AprilTagManager(const rclcpp::NodeOptions& options) : Node("apriltag_manager", options)
    {
      tag_detection_sub = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
        "/apriltag_detections", 10, std::bind(&AprilTagManager::tag_detection_cb, this, _1));

      query_tags_server = rclcpp_action::create_server<QueryAprilTagLog>(
        this,
        "/query_april_tags",
        std::bind(&AprilTagManager::handle_goal, this, _1, _2),
        std::bind(&AprilTagManager::handle_cancel, this, _1),
        std::bind(&AprilTagManager::handle_accepted, this, _1));

      arm_base_link = this->declare_parameter<std::string>("arm_base_link", "link0");
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

  private:
    rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_detection_sub;
    rclcpp_action::Server<QueryAprilTagLog>::SharedPtr query_tags_server;
    std::string arm_base_link;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    
    std::map<int, std::tuple<std_msgs::msg::Header, isaac_ros_apriltag_interfaces::msg::AprilTagDetection>> tag_log;

    void tag_detection_cb(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray & msg)
    {
      // Add/update detections in the tag_log
      for(auto detection : msg.detections)
      {
        std::tuple<std_msgs::msg::Header, isaac_ros_apriltag_interfaces::msg::AprilTagDetection> tag_record;
        tag_record = std::make_tuple(msg.header, detection);
        tag_log[detection.id] = tag_record;
      }
    }

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const QueryAprilTagLog::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received QueryAprilTagLog request");
      if (goal->tag_ids.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Request tag_ids field empty, rejecting request");
        return rclcpp_action::GoalResponse::REJECT;
      } else {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    }

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleQueryAprilTagLog> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleQueryAprilTagLog> goal_handle)
    {
      std::thread{std::bind(&AprilTagManager::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleQueryAprilTagLog> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<QueryAprilTagLog::Result>();
      
      for (int id : goal->tag_ids)
      {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        
        // Retrieve the latest detection of each requested tag id
        if (!(tag_log.find(id) == tag_log.end()))
        {
          xavbot_interfaces::msg::AprilTagDetectionStamped latest_detection;
          latest_detection.header = std::get<0>(tag_log[id]);
          latest_detection.detection = std::get<1>(tag_log[id]);
          

          // Look up tf
          std::string tag_frame;
          tag_frame = std::get<1>(tag_log[id]).family + ":" + std::to_string(id);
          geometry_msgs::msg::TransformStamped t;
          try {
            t = tf_buffer->lookupTransform(arm_base_link.c_str(), tag_frame.c_str(), tf2::TimePointZero);
            if (-0.2 <= t.transform.translation.x && t.transform.translation.x <= 0
              && -0.005 <= t.transform.translation.z && t.transform.translation.z <= 0.005) {
              latest_detection.reachable = true;
            } else {
              latest_detection.reachable = false;
            }
          } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              arm_base_link.c_str(), tag_frame.c_str(), ex.what());
          }

          result->detections.push_back(latest_detection);

        } else {
          result->failed_tags.push_back(id);
        }
      }

      // Return result
      if (!result->detections.empty()){
        RCLCPP_INFO(this->get_logger(), 
          "Success, %ld/%ld requested AprilTags retrieved", result->detections.size(), goal->tag_ids.size());
        goal_handle->succeed(result);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed, none of the requested tag_ids have been logged");
        goal_handle->abort(result);
      }      
    }
}; // class AprilTagManager
} // namespace xavbot_perception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xavbot_perception::AprilTagManager)

