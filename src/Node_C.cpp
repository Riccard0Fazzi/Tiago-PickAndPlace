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
// import for gripper
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <random>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class PickingActionServer {
private:
    
    // CLASS VARIABLES
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2425_group_24_a2::manipulationAction> as_; // action server to the client in Node_B
    std::string action_name_; // name of the server
    ir2425_group_24_a2::manipulationFeedback feedback_; // feedback of the server
    ir2425_group_24_a2::manipulationResult result_; // result of the server
    TrajectoryClient gripper_client; // client to close the gripper
    // moveIt interface variables
    moveit::planning_interface::MoveGroupInterface move_group; 
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

public:

    // [CONSTRUCTOR]
    PickingActionServer(const std::string& name) :
        as_(nh_, name, boost::bind(&PickingActionServer::executeCB, this, _1), false),
        action_name_(name),
        move_group("arm"), 
        gripper_client("/parallel_gripper_controller/follow_joint_trajectory", true) {
            as_.start();
            gripper_client.waitForServer();
            // Change the end effector frame to perform the picking operation
            move_group.setEndEffectorLink("gripper_base_link");
            move_group.setPlanningTime(15.0);
            move_group.setNumPlanningAttempts(5);
    }

    // [CALLBACK] to start the picking action
    // __________________________________________________________________
    void executeCB(const ir2425_group_24_a2::manipulationGoalConstPtr& goal) {
        ROS_INFO("Start Picking of object ID=%d", goal->ID);
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

    // [METHOD] to set Tiago's arm at an initial configuration
    // _______________________________________________________________________
    void initial_config(){
        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.858; // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.113;   // Elbow joint
        initial_joint_positions["arm_4_joint"] = 0.766; // Forearm
        initial_joint_positions["arm_5_joint"] = -1.570;   // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 1.370;  // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;   // End-effector roll
        move_group.setJointValueTarget(initial_joint_positions);  
        auto success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to plan motion to initial configuration.");
            return;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute motion to initial configuration.");
            return;
        }
        ROS_INFO("Initial configuration set successfully.");
    }

    // [METHOD] to set Tiago's arm at a safe configuration for navigation
    // _______________________________________________________________________
    void tuck_config(){
        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.449; // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.029;   // Elbow joint
        initial_joint_positions["arm_4_joint"] = 2.147; // Forearm
        initial_joint_positions["arm_5_joint"] = -2.029;   // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 0.129;  // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;   // End-effector roll
        move_group.setJointValueTarget(initial_joint_positions);  
        auto success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to plan motion to initial configuration.");
            return;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute motion to initial configuration.");
            return;
        }
        ROS_INFO("Initial configuration set successfully.");
    }

    // [METHOD] to approach the object to pick
    // _______________________________________________________________________
    void approach(geometry_msgs::Pose& pose, int id){
        // Set the target pose above the marker (10 cm above)
        // Set the orientation for z-axis pointing downwards
        tf2::Quaternion orientation_downwards;
        if(id <= 3) pose.position.z += 0.15;
        else pose.position.z += 0.25;
        tf2::Quaternion current_orientation(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        // Define the rotation about the x-axis (180°)
        tf2::Quaternion rotation_about_x;
        rotation_about_x.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°
        // Combine the current orientation with the rotation
        tf2::Quaternion combined_orientation = current_orientation * rotation_about_x;
        combined_orientation.normalize(); // Ensure the quaternion is normalized
        // Assign the combined orientation back to the target pose
        pose.orientation.x = combined_orientation.x();
        pose.orientation.y = combined_orientation.y();
        pose.orientation.z = combined_orientation.z();
        pose.orientation.w = combined_orientation.w();  
        bool is_within_bounds = move_group.setPoseTarget(pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return;
        }
        // Plan and execute the motion to the target pose
        auto success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to plan motion to target position.");
            return;
        }
        success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute motion to target position.");
            return;
        }
        ROS_INFO("Approaching completed.");
    }

    // [METHOD] to reach the object through a linear movement
    // _______________________________________________________________________
    void reach(geometry_msgs::Pose& pose){
        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;
        // Ad`d 0.25m offset along the z-axis
        target_pose.position.z -= 0.20;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose);
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Step size for end-effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        auto success = (move_group.execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute Cartesian path.");
            return;
        }
        ROS_INFO("Reaching executed...");
    }

    // [METHOD] to grasp the object by closing the gripper
    // _______________________________________________________________________
    void grasp(){
        // Create the trajectory goal
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("gripper_joint"); // Replace with your gripper joint name

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.0); // Fully closed position
        point.time_from_start = ros::Duration(1.0); // Adjust time for motion

        goal.trajectory.points.push_back(point);

        ROS_INFO("Sending gripper close trajectory goal...");
        gripper_client.sendGoal(goal);

        auto success = gripper_client.waitForResult(ros::Duration(5.0));
        if (success) {
            ROS_INFO("Gripper closed successfully.");
        } else {
            ROS_ERROR("Failed to close gripper.");
        }
        ROS_INFO("Object grasped!");
    }

    // [METHOD] to attach the object to the e-e link
    // _______________________________________________________________________
    void attach(int32_t id) {
        ros::ServiceClient client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        gazebo_ros_link_attacher::Attach srv;
        srv.request.model_name_1 = "tiago";  // Name of the robot (gripper)
        std::string name;
        std::string link_name;
        if(id <= 3){
            // hexagons
            if(id == 1){
                name = "Hexagon";
                link_name = "Hexagon_link";
            }
            else{
                name = "Hexagon_" + std::to_string(id);
                link_name = "Hexagon_" + std::to_string(id) + "_link";
            }
        }
        else if(id <= 6){
            // cubes
            if(id == 4){
                name = "cube";
                link_name = "cube_link";
            }
            else{
                name = "cube_" + std::to_string(id);
                link_name = "cube_" + std::to_string(id) + "_link";
            }
        }
        else{
            // triangle
            if(id == 7){
                name = "Triangle";
                link_name = "Triangle_link";
            }
            else{
                name = "Triangle_" + std::to_string(id);
                link_name = "Triangle_" + std::to_string(id) + "_link";
            }
        }
        srv.request.model_name_2 = name; // Object to be attached
        srv.request.link_name_1 = "arm_7_link";  // Link name of the gripper
        srv.request.link_name_2 = link_name;   // Link name of the object

        if (client.call(srv)) {
            ROS_INFO("Successfully attached object to the gripper.");
            return;
        } else {
            ROS_ERROR("Failed to attach object to the gripper.");
            return;
        }
    }

    // [METHOD] to depart with the object
    // _______________________________________________________________________
    void depart(geometry_msgs::Pose& pose){
        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;
        // Add 0.25m offset along the z-axis
        target_pose.position.z += 0.20;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose);
        waypoints.push_back(target_pose);
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Step size for end-effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        auto success = (move_group.execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (!success) {
            ROS_ERROR("Failed to execute Cartesian path.");
            return;
        }
        ROS_INFO("Departed!");
    }



    // [METHOD] to globally handle the picking operation
    // _______________________________________________________________________
    bool performPicking(int32_t id, geometry_msgs::Pose pose) {
        ROS_INFO("Starting picking operation for ID=%d", id);
        initial_config();
        approach(pose, id);
        ros::Duration(1.0).sleep();
        reach(pose);
        ros::Duration(1.0).sleep();
        // Remove the target object from the collision objects
        std::string object_id = std::to_string(id);
        std::vector<std::string> object_ids = {object_id};
        planning_scene.removeCollisionObjects(object_ids);
        // ROS lINK ATTACHER
        attach(id);
        
        grasp();

        depart(pose);

        initial_config();

        tuck_config();

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
