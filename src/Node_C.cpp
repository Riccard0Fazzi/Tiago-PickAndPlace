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

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> // For TF message

class PickingActionServer {
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2425_group_24_a2::manipulationAction> as_;
    std::string action_name_;
    ir2425_group_24_a2::manipulationFeedback feedback_;
    ir2425_group_24_a2::manipulationResult result_;
    moveit::planning_interface::MoveGroupInterface move_group_; // Interface for controlling the robot arm
    moveit::planning_interface::PlanningSceneInterface planning_scene_; // Interface for planning scene management
    tf2_ros::TransformBroadcaster tf_broadcaster_;

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

        // Perform picking operation
        bool success = performPicking(goal->ID, goal->pose);

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
    bool performPicking(int32_t id, geometry_msgs::Pose pose) {

        ROS_INFO("Starting picking operation for ID=%d", id);

        // Initialize MoveIt interfaces
        moveit::planning_interface::MoveGroupInterface move_group("arm");
        // Change the end effector frame to perform the picking operation
        move_group.setEndEffectorLink("gripper_base_link");
        moveit::planning_interface::PlanningSceneInterface planning_scene;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // change workspace dimensions
        // (min_x, min_y, min_z, max_x, max_y, max_z)
        move_group.setWorkspace(-1.0, -3.0, 0.0, 2.0, 3.0, 3.0);


        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.858; // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.113;   // Elbow joint
        initial_joint_positions["arm_4_joint"] = 0.766; // Forearm
        initial_joint_positions["arm_5_joint"] = -1.570;   // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 1.370;  // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;   // End-effector roll
        /*initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.505;   // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.113;  // Elbow joint
        initial_joint_positions["arm_4_joint"] = 0.832;   // Forearm
        initial_joint_positions["arm_5_joint"] = -1.691;  // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 1.230;   // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;     // End-effector roll*/
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

        // Set the target pose above the marker (10 cm above)
        // Set the orientation for z-axis pointing downwards
        tf2::Quaternion orientation_downwards;
        pose.position.z += 0.35;
        tf2::Quaternion current_orientation(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);

        // Step 3: Define the rotation about the x-axis (180°)
        tf2::Quaternion rotation_about_x;
        rotation_about_x.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°

        // Step 4: Combine the current orientation with the rotation
        tf2::Quaternion combined_orientation = current_orientation * rotation_about_x;
        combined_orientation.normalize(); // Ensure the quaternion is normalized

        // Step 5: Assign the combined orientation back to the target pose
        pose.orientation.x = combined_orientation.x();
        pose.orientation.y = combined_orientation.y();
        pose.orientation.z = combined_orientation.z();
        pose.orientation.w = combined_orientation.w(); 
        
        bool is_within_bounds = move_group.setPoseTarget(pose);
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
