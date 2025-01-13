#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2425_group_24_a2/manipulationAction.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2 conversions
#include <tf2_ros/transform_broadcaster.h> // For TF broadcasting
#include <geometry_msgs/TransformStamped.h> // For TF message
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
    tf2_ros::TransformBroadcaster tf_broadcaster_; // TF broadcaster

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
        // PUBLISH THE FRAME HERE
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "base_footprint"; // Parent frame 
        transform_stamped.child_frame_id = "collision_object_pose"; // Frame name for visualization

        // Set the translation and rotation from the pose of the collision object
        transform_stamped.transform.translation.x = pose.position.x;
        transform_stamped.transform.translation.y = pose.position.y;
        transform_stamped.transform.translation.z = pose.position.z;
        transform_stamped.transform.rotation = pose.orientation;

        // Publish the transformation
        tf_broadcaster_.sendTransform(transform_stamped);

        // Initialize MoveIt interfaces
        moveit::planning_interface::MoveGroupInterface move_group("arm");
        moveit::planning_interface::PlanningSceneInterface planning_scene;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // To print the reference frame for the planning
        ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
        // change workspace dimensions
        //                     (min_x, min_y, min_z, max_x, max_y, max_z)
        move_group.setWorkspace(-1.0, -3.0, 0.0, 2.0, 3.0, 3.0);


        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.505;   // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.113;  // Elbow joint
        initial_joint_positions["arm_4_joint"] = 0.832;   // Forearm
        initial_joint_positions["arm_5_joint"] = -1.691;  // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 1.230;   // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;     // End-effector roll
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
        geometry_msgs::Pose target_pose = pose;
        target_pose.position.z += 0.1;
        // Set the orientation for z-axis pointing downwards
        tf2::Quaternion orientation_downwards;
        orientation_downwards.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°
        // Set the orientation for z-axis pointing downwards
        target_pose.orientation.x = orientation_downwards.x();
        target_pose.orientation.y = orientation_downwards.y();
        target_pose.orientation.z = orientation_downwards.z();
        target_pose.orientation.w = orientation_downwards.w();  
        bool is_within_bounds = move_group.setPoseTarget(target_pose);
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
