#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2425_group_24_a2/manipulationAction.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2 conversions
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
        // Base joint remains centered (no rotation).
        initial_joint_positions["arm_1_joint"] = 0.070;
        // Shoulder joint slightly raised to align the arm above the table and provide a 90-degree angle between the forearm and shoulder.
        initial_joint_positions["arm_2_joint"] = 0.505;
        // Elbow joint 
        initial_joint_positions["arm_3_joint"] = -0.113;  
        // Forearm remains aligned without rotation.
        initial_joint_positions["arm_4_joint"] = 0.832;
        // Wrist pitch adjusted to align the end-effector parallel to the table and point downward.
        initial_joint_positions["arm_5_joint"] = -1.691;  // Approx. -90 degrees in radians.
        // Wrist yaw set to neutral for simplicity.
        initial_joint_positions["arm_6_joint"] = 1.230;
        // End-effector roll neutral for alignment over the table.
        initial_joint_positions["arm_7_joint"] = 0.0;
        move_group.setJointValueTarget(initial_joint_positions);  
        move_group.setPlanningTime(10.0); // Increase to 10 seconds or more
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
        geometry_msgs::Pose inital_pose = pose;
        inital_pose.position.x = 7.88904;
        inital_pose.position.y = -3.1;
        inital_pose.position.z = 1.0;
        // Set the orientation for z-axis pointing downwards
        tf2::Quaternion orientation_downwards;
        orientation_downwards.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°
        inital_pose.orientation.x = orientation_downwards.x();
        inital_pose.orientation.y = orientation_downwards.y();
        inital_pose.orientation.z = orientation_downwards.z();
        inital_pose.orientation.w = orientation_downwards.w(); 
        bool is_within_bounds = move_group.setPoseTarget(inital_pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return false;
        }
        move_group.setPlanningTime(10.0); // Increase to 10 seconds or more
        ROS_INFO("Setting initial configuration for Tiago's arm...");
        success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
        // Set the orientation for z-axis pointing downwards
        target_pose.orientation.x = orientation_downwards.x();
        target_pose.orientation.y = orientation_downwards.y();
        target_pose.orientation.z = orientation_downwards.z();
        target_pose.orientation.w = orientation_downwards.w(); 
        is_within_bounds = move_group.setPoseTarget(target_pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return false;
        }
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
