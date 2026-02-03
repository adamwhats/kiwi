#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

// Custom service definition - you'll need to create this
// For now, I'll define it inline, but you should create a proper .srv file
struct MoveToRequest {
    geometry_msgs::msg::PoseStamped target_pose;  // Target for gripper_tcp
    std::string gripper_state;                    // "open" or "closed"
    bool plan_only;                               // For testing/debugging
};

struct MoveToResponse {
    bool success;
    std::string message;
    double planning_time;
    double execution_time;
};

class MobileManipulationService : public rclcpp::Node
{
public:
    MobileManipulationService() : Node("mobile_manipulation_service"),
                                  tf_buffer_(this->get_clock()),
                                  tf_listener_(tf_buffer_)
    {
        // Initialize MoveIt interfaces
        move_group_combined_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "combined");
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "arm");
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "gripper");

        // Set planning parameters
        move_group_combined_->setPlanningTime(10.0);
        move_group_combined_->setNumPlanningAttempts(5);
        move_group_arm_->setPlanningTime(10.0);
        move_group_gripper_->setPlanningTime(5.0);

        // Nav2 action client
        nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            this, "navigate_through_poses");

        // Service server - using SetBool as placeholder, you should create custom service
        // For now, I'll create a more complex service structure
        move_to_service_ = this->create_service<std_srvs::srv::SetBool>(
            "move_to_pose", 
            std::bind(&MobileManipulationService::handle_move_to_service, 
                      this, std::placeholders::_1, std::placeholders::_2));

        // Parameters
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("world_frame", "map");
        this->declare_parameter("planning_frame", "odom");
        
        base_frame_ = this->get_parameter("base_frame").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        planning_frame_ = this->get_parameter("planning_frame").as_string();

        RCLCPP_INFO(this->get_logger(), "Mobile Manipulation Service initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        nav2_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Nav2 action server connected");
    }

    // Main service call - you should replace this with your custom service type
    void handle_move_to_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // This is a placeholder - you'll want to create a proper service definition
        // For demonstration, I'll show how to call the main function
        
        // Example target pose (you'd get this from the actual service request)
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.header.stamp = this->now();
        target_pose.pose.position.x = 1.0;  // Example values
        target_pose.pose.position.y = 0.5;
        target_pose.pose.position.z = 0.3;
        target_pose.pose.orientation.w = 1.0;
        
        std::string gripper_state = request->data ? "open" : "closed";
        
        auto result = execute_mobile_manipulation(target_pose, gripper_state, false);
        response->success = result.success;
        response->message = result.message;
    }

    MoveToResponse execute_mobile_manipulation(const geometry_msgs::msg::PoseStamped& target_pose,
                                              const std::string& gripper_state,
                                              bool plan_only = false)
    {
        MoveToResponse response;
        auto start_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Planning mobile manipulation to pose [%.3f, %.3f, %.3f] with gripper: %s",
                    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                    gripper_state.c_str());

        // Step 1: Set gripper state for planning
        if (!set_gripper_state(gripper_state)) {
            response.success = false;
            response.message = "Failed to set gripper state: " + gripper_state;
            return response;
        }

        // Step 2: Plan combined motion (base + arm + gripper)
        move_group_combined_->setPoseTarget(target_pose, "gripper_tcp");
        
        moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
        auto planning_result = move_group_combined_->plan(combined_plan);
        
        response.planning_time = (this->now() - start_time).seconds();
        
        if (planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            response.success = false;
            response.message = "Combined motion planning failed";
            RCLCPP_ERROR(this->get_logger(), "%s", response.message.c_str());
            return response;
        }

        RCLCPP_INFO(this->get_logger(), "Combined motion planned successfully");

        if (plan_only) {
            response.success = true;
            response.message = "Planning successful (plan_only=true)";
            return response;
        }

        // Step 3: Extract trajectories
        auto base_poses = extract_base_poses_from_trajectory(combined_plan.trajectory_);
        auto arm_trajectory = extract_arm_trajectory(combined_plan.trajectory_);

        // Step 4: Execute base motion with Nav2 (with obstacle avoidance)
        auto execution_start_time = this->now();
        
        if (!base_poses.empty()) {
            RCLCPP_INFO(this->get_logger(), "Executing base motion through %zu poses with Nav2", 
                        base_poses.size());
            
            if (!execute_base_motion_nav2(base_poses)) {
                response.success = false;
                response.message = "Base motion execution failed";
                return response;
            }
            RCLCPP_INFO(this->get_logger(), "Base motion completed");
        }

        // Step 5: Execute arm motion
        if (!arm_trajectory.joint_trajectory.points.empty()) {
            RCLCPP_INFO(this->get_logger(), "Executing arm motion");
            
            if (!execute_arm_motion(arm_trajectory)) {
                response.success = false;
                response.message = "Arm motion execution failed";
                return response;
            }
            RCLCPP_INFO(this->get_logger(), "Arm motion completed");
        }

        response.execution_time = (this->now() - execution_start_time).seconds();
        response.success = true;
        response.message = "Mobile manipulation completed successfully";
        
        RCLCPP_INFO(this->get_logger(), "Mobile manipulation completed in %.2fs (planning: %.2fs, execution: %.2fs)",
                    (this->now() - start_time).seconds(), response.planning_time, response.execution_time);
        
        return response;
    }

private:
    // MoveIt interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_combined_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    
    // Nav2 client
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav2_client_;
    
    // Service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_to_service_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    std::string base_frame_;
    std::string world_frame_;
    std::string planning_frame_;

    bool set_gripper_state(const std::string& state)
    {
        if (state != "open" && state != "closed") {
            RCLCPP_ERROR(this->get_logger(), "Invalid gripper state: %s. Use 'open' or 'closed'", state.c_str());
            return false;
        }

        move_group_gripper_->setNamedTarget(state);
        
        // Apply this state to the combined planning group as well
        move_group_combined_->setNamedTarget(state);
        
        RCLCPP_INFO(this->get_logger(), "Set gripper state to: %s", state.c_str());
        return true;
    }

    std::vector<geometry_msgs::msg::PoseStamped> extract_base_poses_from_trajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        
        // Find virtual_joint index - for planar joint, it should have 3 values [x, y, theta]
        int virtual_joint_idx = -1;
        for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
            if (trajectory.joint_trajectory.joint_names[i] == "virtual_joint/x" ||
                trajectory.joint_trajectory.joint_names[i] == "virtual_joint") {
                virtual_joint_idx = i;
                break;
            }
        }
        
        if (virtual_joint_idx == -1) {
            RCLCPP_WARN(this->get_logger(), "No virtual_joint found in trajectory");
            return poses;
        }

        // Convert joint trajectory points to poses
        // Note: For planar joints, MoveIt typically uses [x, y, theta] representation
        for (const auto& point : trajectory.joint_trajectory.points) {
            if (point.positions.size() <= virtual_joint_idx + 2) {
                continue;  // Skip if not enough position data
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = world_frame_;
            pose.header.stamp = this->now();
            
            // For planar joint: positions[virtual_joint_idx] = x, +1 = y, +2 = theta
            pose.pose.position.x = point.positions[virtual_joint_idx];
            pose.pose.position.y = point.positions[virtual_joint_idx + 1];
            pose.pose.position.z = 0.0;
            
            // Convert theta to quaternion
            double theta = point.positions[virtual_joint_idx + 2];
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            pose.pose.orientation = tf2::toMsg(q);
            
            poses.push_back(pose);
        }
        
        RCLCPP_INFO(this->get_logger(), "Extracted %zu base poses from trajectory", poses.size());
        return poses;
    }

    moveit_msgs::msg::RobotTrajectory extract_arm_trajectory(
        const moveit_msgs::msg::RobotTrajectory& original_trajectory)
    {
        moveit_msgs::msg::RobotTrajectory arm_traj;
        arm_traj.joint_trajectory.header = original_trajectory.joint_trajectory.header;
        
        // Extract only arm joints (link1_joint, link2_joint, link3_joint based on your controller config)
        std::vector<int> arm_indices;
        std::vector<std::string> arm_joint_names = {"link1_joint", "link2_joint", "link3_joint"};
        
        for (const auto& arm_joint : arm_joint_names) {
            for (size_t i = 0; i < original_trajectory.joint_trajectory.joint_names.size(); ++i) {
                if (original_trajectory.joint_trajectory.joint_names[i] == arm_joint) {
                    arm_indices.push_back(i);
                    arm_traj.joint_trajectory.joint_names.push_back(arm_joint);
                    break;
                }
            }
        }
        
        // Copy trajectory points for arm joints only
        for (const auto& point : original_trajectory.joint_trajectory.points) {
            trajectory_msgs::msg::JointTrajectoryPoint new_point;
            new_point.time_from_start = point.time_from_start;
            
            for (int idx : arm_indices) {
                if (idx < static_cast<int>(point.positions.size())) {
                    new_point.positions.push_back(point.positions[idx]);
                }
                if (idx < static_cast<int>(point.velocities.size())) {
                    new_point.velocities.push_back(point.velocities[idx]);
                }
                if (idx < static_cast<int>(point.accelerations.size())) {
                    new_point.accelerations.push_back(point.accelerations[idx]);
                }
            }
            
            arm_traj.joint_trajectory.points.push_back(new_point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Extracted arm trajectory with %zu points", 
                    arm_traj.joint_trajectory.points.size());
        return arm_traj;
    }

    bool execute_base_motion_nav2(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        auto nav_goal = std::make_shared<nav2_msgs::action::NavigateThroughPoses::Goal>();
        nav_goal->poses = poses;
        nav_goal->behavior_tree = "";  // Use default BT
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        
        // Add feedback callback to monitor progress
        send_goal_options.feedback_callback = 
            [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
                   const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback) {
                RCLCPP_DEBUG(this->get_logger(), "Nav2 progress: %d/%zu poses completed", 
                            feedback->current_pose + 1, feedback->number_of_poses);
            };
        
        auto future = nav2_client_->async_send_goal(nav_goal, send_goal_options);
        
        // Wait for goal acceptance
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(5)) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2");
            return false;
        }

        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected");
            return false;
        }

        // Wait for result
        auto result_future = goal_handle->async_get_result();
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get Nav2 result");
            return false;
        }

        auto result = result_future.get();
        return result.code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    bool execute_arm_motion(const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        // Execute the arm trajectory using MoveIt
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        arm_plan.trajectory_ = trajectory;
        
        auto result = move_group_arm_->execute(arm_plan);
        return result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    // Use MultiThreadedExecutor to handle concurrent operations
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MobileManipulationService>();
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Mobile Manipulation Service ready");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}