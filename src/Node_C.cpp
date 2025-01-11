#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2425_group_24_a2/manipulationAction.h>
#include <geometry_msgs/Pose.h>
// import for MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


class PickingActionServer {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2425_group_24_a2::manipulationAction> as_;
    std::string action_name_;
    ir2425_group_24_a2::manipulationFeedback feedback_;
    ir2425_group_24_a2::manipulationResult result_;
    moveit::planning_interface::MoveGroupInterface move_group_; // Interface for controlling the robot arm
    moveit::planning_interface::PlanningSceneInterface planning_scene_; // Interface for planning scene management

public:
    PickingActionServer(const std::string& name) :
        as_(nh_, name, boost::bind(&PickingActionServer::executeCB, this, _1), false),
        action_name_(name), 
        move_group_("arm") {
            as_.start();
            ROS_INFO("Picking Action Server initialized.");
    }

    // CALLBACK that receives the goal when sent
    void executeCB(const ir2425_group_24_a2::manipulationGoalConstPtr& goal) {
        ROS_INFO("Received goal: ID=%d", goal->ID);
        geometry_msgs::Pose target_pose = goal->pose;

        // Perform picking operation
        bool success = performPicking(goal->ID, target_pose);

        if (success) {
            result_.succeeded = true;
            ROS_INFO("Picking action succeeded.");
            as_.setSucceeded(result_);
        } else {
            result_.succeeded = false;
            ROS_WARN("Picking action failed.");
            as_.setAborted(result_);
        }
    }

private:

    // method to perform the picking operation
    bool performPicking(int32_t id, const geometry_msgs::Pose& pose) {
        ROS_INFO("Starting picking operation for ID=%d", id);

        // Initialize MoveIt interfaces
        moveit::planning_interface::MoveGroupInterface move_group("arm");
        moveit::planning_interface::PlanningSceneInterface planning_scene;
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.0;       // Base joint remains centered (no rotation)
        initial_joint_positions["arm_2_joint"] = 0.0;       // Shoulder joint at neutral (higher than -0.5)
        initial_joint_positions["arm_3_joint"] = 0.6;       // Elbow joint moderately extended
        initial_joint_positions["arm_4_joint"] = 0.0;       // No rotation at the forearm
        initial_joint_positions["arm_5_joint"] = -1.2;      // Wrist pitch to direct the end-effector downward
        initial_joint_positions["arm_6_joint"] = 0.0;       // Neutral wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;       // Neutral end-effector roll

        move_group.setJointValueTarget(initial_joint_positions);
        ROS_INFO("Setting initial configuration for Tiago's arm...");
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to plan motion to initial configuration.");
            return false;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute motion to initial configuration.");
            return false;
        }
        ROS_INFO("Initial configuration set successfully.");

        // Set the target pose above the marker (10 cm above)
        geometry_msgs::Pose target_pose = pose;
        target_pose.position.z += 0.1; 
        bool is_within_bounds = move_group.setPoseTarget(target_pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return false;
        }
        move_group.setPlanningTime(10.0); // Increase to 10 seconds or more
        // Plan and execute the motion to the target pose
        ROS_INFO("Planning motion to target position above the marker...");
        success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to plan motion to target position.");
            return false;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute motion to target position.");
            return false;
        }
        ROS_INFO("Motion to target position completed.");

        // Optional: Adjust for side grasp if the object is a hexagon
        // This part requires object type identification and recalculating the target pose.

        // Perform linear movement to grasp the object
        geometry_msgs::Pose grasp_pose = pose; // Align with the marker center
        move_group.setPoseTarget(grasp_pose);

        ROS_INFO("Planning linear motion for grasping...");
        success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success) {
            ROS_ERROR("Failed to plan linear motion for grasping.");
            return false;
        }

        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute linear motion for grasping.");
            return false;
        }

        ROS_INFO("Grasping motion completed.");

        // Simulate gripper closing (adjust gripper interface as per your robot)
        ROS_INFO("Closing the gripper...");
        // Example: send command to gripper action server

        // Remove the target object from the collision objects
        std::string object_id = std::to_string(id);
        std::vector<std::string> object_ids = {object_id};
        planning_scene.removeCollisionObjects(object_ids);
        ROS_INFO("Removed target object from collision objects.");

        ROS_INFO("Picking operation completed for ID=%d", id);
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "picking_action_server");
    PickingActionServer server("/picking");
    ros::spin();
    return 0;
}
