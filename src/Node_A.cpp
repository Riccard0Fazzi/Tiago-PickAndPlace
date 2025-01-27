#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> 
#include <move_base_msgs/MoveBaseAction.h> // to navigate Tiago
#include <tiago_iaslab_simulation/Coeffs.h> // to request m,q from /straight_line_srv
#include <ir2425_group_24_a2/detection.h>
#include <ir2425_group_24_a2/picking_completed.h>
// import fro camera tilt
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// import for ObjectDetectionCallback
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
// import for TF
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> // For TF message

// import for MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <gazebo_ros_link_attacher/Attach.h>


// Alias for the move_base action client

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class NodeA 
{
public:

    bool picked_object;
    bool activated;

    // CONSTRUCTOR
    NodeA(ros::NodeHandle& nh)
        : nh_(nh),
          client_(nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv")), // service client to the straight line service
          move_base_client_("/move_base", true), // action client to the move_base action server
          head_client("/head_controller/follow_joint_trajectory", true),
          torso_client_("/torso_controller/follow_joint_trajectory", true),
          tf_listener(tf_buffer),
          attach_client(nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach")),
          gripper_client("/parallel_gripper_controller/follow_joint_trajectory", true),
          move_group("arm")

    {
        ROS_INFO("Node A initialized and ready to call /straight_line_srv service.");
        // Wait for the move_base action server to start
        ROS_INFO("Waiting for the /move_base action server to start...");
        move_base_client_.waitForServer();
        ROS_INFO("/move_base action server started.");
		// publisher for the initial tilt of the camera 
		// to be ready to detect aprilTags
		tilt_cam_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
		detection_pub = nh.advertise<ir2425_group_24_a2::detection>("/start_detection", 10);
		picking_sub = nh.subscribe("/picking_terminated", 10, &NodeA::PickingTerminatedCallBack, this);
        // Wait for the action server to be available
        ROS_INFO("Waiting for head action server to start...");
        head_client.waitForServer();
         // Wait for the action server to start
        ROS_INFO("Waiting for torso controller action server...");
        torso_client_.waitForServer();
        ROS_INFO("Connected to torso controller action server.");
        ROS_INFO("Waiting for gripper action server...");
        gripper_client.waitForServer();
        move_group.setEndEffectorLink("gripper_base_link");
		initialize_routines(); 
        move_group.setPlanningTime(15.0);
        move_group.setNumPlanningAttempts(5);
        picked_object = false;
        activated = false;
    }

	void PickingTerminatedCallBack(const ir2425_group_24_a2::picking_completed::ConstPtr& msg)
	{
		if(msg->picking_completed)
		{
			ROS_INFO("Picking completed!");
			ROS_INFO("Starting placing procedure");
            picked_object = true;
            id = msg->id;	// saving the ID of the object that will be used to detach		
            return;
		}
	}

    // CallBack for Object Detection
    // ______________________________________________
    // Callback that continuously search for AprilTags
    // in Tiago's camera and compute their position
    // wrt map frame (20 Hz)
    void AprilTagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) 
    {
        if(msg->detections.empty()) return;
        // callBack always running, saving detected poses only when required
        for(const auto& detection : msg->detections)
        {
            int id_tag = detection.id[0];
            if(id_tag==10 && line_origin.header.frame_id.empty())
            {
                ROS_INFO("Detected object ID: %d",id_tag);
                // get the pose of the object
                geometry_msgs::PoseStamped frame;
                frame.header.seq = static_cast<uint32_t>(id_tag); // saving ID
                //frame.header.stamp = detection.pose.header.stamp;       // Use detection timestamp
                frame.header.frame_id = detection.pose.header.frame_id; // Get frame_id from the detection
                frame.pose.position = detection.pose.pose.pose.position; // saving position
                frame.pose.orientation = detection.pose.pose.pose.orientation;// saving orientation
                // Define a transform from the camera frame (or detected frame) to the map frame
                ros::Rate rate(100.0);  // Loop frequency in Hz
                // transform from camera frame to map frame
                // PUBLISH THE FRAME HERE
                geometry_msgs::TransformStamped transform_stamped;
                transform_stamped.header.stamp = ros::Time::now();
                transform_stamped.header.frame_id = detection.pose.header.frame_id; // Parent frame 
                transform_stamped.child_frame_id = "table_origin"; // Frame name for visualization

                // Set the translation and rotation from the pose of the collision object
                transform_stamped.transform.translation.x = frame.pose.position.x;
                transform_stamped.transform.translation.y = frame.pose.position.y;
                transform_stamped.transform.translation.z = frame.pose.position.z;
                transform_stamped.transform.rotation = frame.pose.orientation;

                // Publish the transformation
                tf_broadcaster_.sendTransform(transform_stamped);
                ros::spinOnce();

                while (ros::ok()) 
                {
                    ROS_INFO("Trying to do the transformation!!");
                    // modified frame.header.stamp in place of ros::Time::now()
                    if(tf_buffer.canTransform("base_footprint", frame.header.frame_id, ros::Time::now(), ros::Duration(1.0))) {
                        try {
                            tf_buffer.transform(frame, line_origin, "base_footprint", ros::Duration(0.1));
                            break;  // Exit loop after successful transformation
                        } catch (tf2::TransformException &ex) {
                        ROS_WARN("Could not transform pose from %s to base_footprint frame: %s", frame.header.frame_id.c_str(), ex.what());
                        }
                    }
                    rate.sleep();    
                }	

                // PUBLISH THE FRAME HERE
                geometry_msgs::TransformStamped transform_stamped_2;
                transform_stamped_2.header.stamp = ros::Time::now();
                transform_stamped_2.header.frame_id = "base_footprint"; // Parent frame 
                transform_stamped_2.child_frame_id = "table_origin_in_bf"; // Frame name for visualization

                // Set the translation and rotation from the pose of the collision object
                transform_stamped_2.transform.translation.x = line_origin.pose.position.x;
                transform_stamped_2.transform.translation.y = line_origin.pose.position.y;
                transform_stamped_2.transform.translation.z = line_origin.pose.position.z;
                transform_stamped_2.transform.rotation = line_origin.pose.orientation;

                // Publish the transformation
                tf_broadcaster_.sendTransform(transform_stamped_2);
                activated = true;
                ros::spinOnce();
                break;
            }
        }
	}

	void initialize_routines()
    {
        // ROUTINE A
        // ___________________________________________________________
        // (from initial pose to picking pose)
		move_base_msgs::MoveBaseGoal goal; 
		goal.target_pose.header.frame_id = "map"; // Use the map frame
		
		// OUT OF CORRIDOR 
        goal.target_pose.pose.position.x = 6.83904; // x-coordinate
        goal.target_pose.pose.position.y = 0.0; // y-coordinate
        goal.target_pose.pose.position.z = 0.0; // z-coordinate
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = -0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
		routineA.push_back(goal);
 
		// lower WAY-CORNER
        goal.target_pose.pose.position.x = 6.83904; // x-coordinate
        goal.target_pose.pose.position.y = -4.01049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.0; // sin(π)
        goal.target_pose.pose.orientation.w = 1.0; // cos(π) 
		
		routineA.push_back(goal);

        // PICKING POSE
        goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -3.81049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.7372; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.6755; // cos(π/4)
		
		routineA.push_back(goal);

        // ROUTINE B (loop)
        // ________________________________________________________
        // (from picking pose to placing pose and viceversa)

		// PICKING POSE orientation
		goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -3.71049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 1.0; // cos(π/4)
		
		routineB.push_back(goal);

        // upper WAY-CORNER
        goal.target_pose.pose.position.x = 8.83904; // x-coordinate
        goal.target_pose.pose.position.y = -4.01049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
		routineB.push_back(goal);

        // PLACING POSE
        goal.target_pose.pose.position.x = 8.58904; // x-coordinate 8.53904
        goal.target_pose.pose.position.y = -2.15049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.9990; // sin(π/4)
        goal.target_pose.pose.orientation.w = -0.0436; // cos(π/4)

		routineB.push_back(goal);
/*
        // POST PLACING BACKING UP
        goal.target_pose.pose.position.x = 9.03904;; // x-coordinate
        goal.target_pose.pose.position.y = -2.00049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 1.0; // cos(π/4)
		
		routineB.push_back(goal);   


        // POST PLACING ORIENTATION
        goal.target_pose.pose.position.x = 9.03904;; // x-coordinate
        goal.target_pose.pose.position.y = -2.00049; // y-coordinate
        goal.target_pose.pose.orientation.z = -0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
		routineB.push_back(goal);
*/

        goal.target_pose.pose.position.x = 8.8; // x-coordinate
        goal.target_pose.pose.position.y = -1.5; // y-coordinate
        goal.target_pose.pose.orientation.z =  0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)

		routineB.push_back(goal);

        goal.target_pose.pose.position.x = 8.8; // x-coordinate
        goal.target_pose.pose.position.y = 0.0; // y-coordinate
        goal.target_pose.pose.orientation.z = 1.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.0; // cos(π/4)

		routineB.push_back(goal);
/*
        // upper WAY-CORNER
        goal.target_pose.pose.position.x = 8.83904; // x-coordinate
        goal.target_pose.pose.position.y = -4.01049; // y-coordinate
        goal.target_pose.pose.orientation.z = 1.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.0; // cos(π/4)
		
		routineB.push_back(goal);

        // approach Picking Pose
        goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -4.01049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.7372; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.6755; // cos(π/4)
		
		routineB.push_back(goal);

        // PICKING POSE
        goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -3.81049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.7372; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.6755; // cos(π/4)
		
		routineB.push_back(goal);
    */
    }

    void initializeDetection() {
        ir2425_group_24_a2::detection msg;
        

        // Define pan and tilt points
        std::vector<std::pair<double, double>> positions = {
            {0.0, -M_PI / 4.0},        // Look down 50 degrees
            {+M_PI / 7.2, -M_PI / 4.0},        // Look left 25 degrees (maintaining down as 1)
            {-M_PI / 7.2, -M_PI / 4.0},         // Look right 50 degrees (from the previous position, maintaining down as 1)
            {0.0, -M_PI / 4.0}   
        };
        // Iterate through the positions
        for (const auto& pos : positions) {
            // Create a FollowJointTrajectoryGoal message
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

            // Create a trajectory point
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = {pos.first, pos.second};  // Set pan and tilt
            point.time_from_start = ros::Duration(1.0);  // 1 second to reach the position
            goal.trajectory.points.push_back(point);

            // Send the goal
            head_client.sendGoal(goal);
            goal = control_msgs::FollowJointTrajectoryGoal(); // Reset to default
            
            // Monitor the result while allowing callbacks to process
            while (!head_client.waitForResult(ros::Duration(0.1))) {
                ros::spinOnce();  // Process other callbacks (e.g., Object Detection)
            }

            // Check if the goal was successfully executed
            if (head_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_WARN("Failed to move camera to position: pan=%f, tilt=%f", pos.first, pos.second);
            }
            // Wait for one second before moving to the next position
            ros::Duration(1.0).sleep();
            // Publish the message to initialize detection
            msg.collect = true;
            // Publish detection message 
            detection_pub.publish(msg);
            ros::spinOnce();
            while (ros::ok()) {
                if(tf_buffer.canTransform("base_footprint", "xtion_rgb_frame", ros::Time::now(), ros::Duration(1.0))) break;
            }
            ros::Duration(1.0).sleep();
        }
        msg.collect = false;
        // Publish detection message 
        detection_pub.publish(msg);
        ros::spinOnce();
        ros::Duration(1.0).sleep();

        ROS_INFO("Camera movement routine completed.");

    }

    void moveHead(std::vector<std::pair<double, double>> position) {
        // Define pan and tilt points
        for (const auto& pos : position) {
            // Create a FollowJointTrajectoryGoal message
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = {pos.first, pos.second};
            point.time_from_start = ros::Duration(1.0);
            goal.trajectory.points.push_back(point);

            // Send the goal
            head_client.sendGoal(goal);
            
            // Wait for the result
            while (!head_client.waitForResult(ros::Duration(0.1))) {
               // AsyncSpinner
            }

            if (head_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_WARN("Failed to move camera to position: pan=%f, tilt=%f", pos.first, pos.second);
            }
        }
        ROS_INFO("Camera movement routine completed.");
    }

    // GET[m,q] method
    // __________________________________________________
    // method to request the /straight_line_srv service
    // and retrieve the m,q line coefficients for the
    // placing task
    void callLineService()
    {
        // Create a service request and response object
        tiago_iaslab_simulation::Coeffs srv;

        // Set the 'ready' field in the request
        srv.request.ready = true;

        // Call the service
        if (client_.call(srv))
        {
            ROS_INFO("/straight_line_srv service called successfully.");

            // Ensure the response contains at least two coefficients
            if (srv.response.coeffs.size() >= 2)
            {
                // Assign the first two coefficients to m and q
                m = static_cast<double>(srv.response.coeffs[0]);
                q = static_cast<double>(srv.response.coeffs[1]);

                ROS_INFO("Coefficients (m, q): m = %f, q = %f", m, q);
            }
            else
            {
                ROS_ERROR("Response does not contain enough coefficients.");
            }
        }
        else
        {
            ROS_ERROR("Failed to call /straight_line_srv service");
        }
    }

    // PICKING POSE NAVIGATION method
    // __________________________________________________
    // method to navigate Tiago to the picking pose to be
    // able to do the picking task
    void navigateToPickingPose()
    {
		for (size_t i = 0; i < routineA.size(); ++i) {

            routineA[i].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
			// Navigation to the Picking Pose
			ROS_INFO("[Navigation] x = %f, y = %f", routineA[i].target_pose.pose.position.x, routineA[i].target_pose.pose.position.y);

			// Send the goal to move_base
			move_base_client_.sendGoal(routineA[i]);
			// wait for the result
			move_base_client_.waitForResult();
            // wait for stable routine
            ros::Duration(0.5).sleep();
		}
		
		if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The robot reached the Picking Pose successfully.");
		else
			ROS_WARN("The robot failed to reach the Picking Pose.");
    }
    
    // PLACING POSE NAVIGATION method
    // __________________________________________________
    // method to navigate Tiago to the picking pose to be
    // able to do the picking task
    void navigateToPlacingPose()
    {
		for (size_t i = 0; i < 3; ++i) {

            routineB[i].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
			// Navigation to the Picking Pose
			ROS_INFO("[Navigation] x = %f, y = %f", routineB[i].target_pose.pose.position.x, routineB[i].target_pose.pose.position.y);

			// Send the goal to move_base
			move_base_client_.sendGoal(routineB[i]);
			// wait for the result
			move_base_client_.waitForResult();
            // wait for stable routine
            ros::Duration(0.5).sleep();
		}
		
		if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The robot reached the Placing Pose successfully.");
		else
			ROS_WARN("The robot failed to reach the Placing Pose.");
    }
    
    // PICKING POSE NAVIGATION method
    // __________________________________________________
    // method to navigate Tiago to the picking pose to be
    // able to do the picking task
    void navigateBackToPickingPose()
    {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();

        routineB[3].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
		// Navigation to the Picking Pose
		ROS_INFO("[Navigation] x = %f, y = %f", routineB[3].target_pose.pose.position.x, routineB[3].target_pose.pose.position.y);

		// Send the goal to move_base
		move_base_client_.sendGoal(routineB[3]);
		// wait for the result
		move_base_client_.waitForResult();
        // wait for stable routine
        ros::Duration(0.5).sleep();
        routineB[4].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
		// Navigation to the Picking Pose
		ROS_INFO("[Navigation] x = %f, y = %f", routineB[4].target_pose.pose.position.x, routineB[4].target_pose.pose.position.y);

		// Send the goal to move_base
		move_base_client_.sendGoal(routineB[4]);
		// wait for the result
		move_base_client_.waitForResult();
        // wait for stable routine
        ros::Duration(0.5).sleep();

        navigateToPickingPose();
		
		if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The robot reached the Picking Pose successfully.");
		else
			ROS_WARN("The robot failed to reach the Picking Pose.");
        // Stop the spinner after finishing
        spinner.stop();
    }

    void liftTorso() {
        // Define the goal for lifting the torso
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("torso_lift_joint");

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(0.34); // Maximum extension of the torso
        point.time_from_start = ros::Duration(2.0); // 5 seconds to reach the position

        goal.trajectory.points.push_back(point);
        goal.trajectory.header.stamp = ros::Time::now();

        // Send the goal
        ROS_INFO("Sending torso lift goal...");
        torso_client_.sendGoal(goal);

        // Wait for the result
        torso_client_.waitForResult();
        if (torso_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully lifted torso to maximum extension.");
        } else {
            ROS_ERROR("Failed to lift torso.");
        }
    }
    
    void initial_config() {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();

        // Set initial configuration for Tiago's arm
        std::map<std::string, double> initial_joint_positions;
        initial_joint_positions["arm_1_joint"] = 0.070;   // Base joint
        initial_joint_positions["arm_2_joint"] = 0.858;  // Shoulder joint
        initial_joint_positions["arm_3_joint"] = -0.113; // Elbow joint
        initial_joint_positions["arm_4_joint"] = 0.766;  // Forearm
        initial_joint_positions["arm_5_joint"] = -1.570; // Wrist pitch
        initial_joint_positions["arm_6_joint"] = 1.370;  // Wrist yaw
        initial_joint_positions["arm_7_joint"] = 0.0;    // End-effector roll

        if (!move_group.setJointValueTarget(initial_joint_positions)) {
            ROS_ERROR("Invalid or unreachable initial joint configuration.");
            return;
        }

        /*
        ros::Duration timeout(10.0); // 10 seconds timeout
        bool success = false;

        while (ros::ok() && !success) {
            success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (ros::Time::now() - start_time > timeout) {
                ROS_WARN("Motion planning timed out after 10 seconds.");
                break;
            }
            ROS_INFO("Waiting for the initial config...");
            ros::Duration(0.1).sleep(); // Sleep for 100ms
        }


        if (!success) {
            ROS_ERROR("Failed to plan motion to initial configuration.");
            return;
        }

        success = false;
        start_time = ros::Time::now();

        while (ros::ok() && !success) {
            success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (ros::Time::now() - start_time > timeout) {
                ROS_WARN("Motion execution timed out after 10 seconds.");
                break;
            }
            ros::Duration(0.1).sleep(); // Sleep for 100ms
        }

        if (!success) {
            ROS_ERROR("Failed to execute motion to initial configuration.");
            return;
        }

        ROS_INFO("Initial configuration set successfully.");
        */
        
        move_group.setPlanningTime(15.0);
        move_group.setNumPlanningAttempts(5);
        ROS_INFO("Setting initial configuration for Tiago's arm...");
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }
        ROS_INFO("Motion plan successfully generated!");

        ROS_INFO("Executing the motion...");
        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Initial configuration successfully reached!");

        // Stop the spinner after finishing
        spinner.stop();
    }
    
    //  -------------- APPROACH ----------------------
    void approach(geometry_msgs::Pose& pose) {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ROS_INFO("Start approach");

        // Adjust the target pose to be 25 cm above the marker
        pose.position.z += 0.25;

        // Step 1: Extract the current orientation from the pose
        tf2::Quaternion current_orientation(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);

        // Step 2: Define the rotation about the x-axis (180°)
        tf2::Quaternion rotation_about_x;
        rotation_about_x.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°

        // Step 3: Combine the current orientation with the rotation
        tf2::Quaternion combined_orientation = current_orientation * rotation_about_x;
        combined_orientation.normalize(); // Ensure the quaternion is normalized

        // Step 4: Assign the combined orientation back to the target pose
        pose.orientation.x = combined_orientation.x();
        pose.orientation.y = combined_orientation.y();
        pose.orientation.z = combined_orientation.z();
        pose.orientation.w = combined_orientation.w();

        // Publish the transform for visualization
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "base_footprint"; // Parent frame
        transform_stamped.child_frame_id = "placing_pose";    // Frame name for visualization
        transform_stamped.transform.translation.x = pose.position.x;
        transform_stamped.transform.translation.y = pose.position.y;
        transform_stamped.transform.translation.z = pose.position.z;
        transform_stamped.transform.rotation = pose.orientation;

        tf_broadcaster_.sendTransform(transform_stamped);

        // Set the pose target
        bool is_within_bounds = move_group.setPoseTarget(pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return;
        }

        ROS_INFO("Setting initial configuration for Tiago's arm...");
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }
        ROS_INFO("Motion plan successfully generated!");

        ROS_INFO("Executing the motion...");
        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }

        ROS_INFO("Motion to target position completed.");

        // Stop the spinner after finishing
        spinner.stop();
    }


    // method to add the table as a collision object
    void CollisionTable()
    {

        // Table dimensions (slightly larger than real ones for safety)
        double table_size = 0.95; 

        // Pick-up table
        moveit_msgs::CollisionObject pickup_table;
        pickup_table.header.frame_id = "base_footprint";  // Use map as the reference frame
        pickup_table.id = "pickup_table";

        // Define cube dimensions
        shape_msgs::SolidPrimitive pickup_table_primitive;
        pickup_table_primitive.type = shape_msgs::SolidPrimitive::BOX;
        pickup_table_primitive.dimensions.resize(3);
        pickup_table_primitive.dimensions[0] = table_size; // Length
        pickup_table_primitive.dimensions[1] = table_size; // Width
        pickup_table_primitive.dimensions[2] = 0.78; // Height

        // Define the pose
        geometry_msgs::Pose pickup_table_pose;
        //pickup_table_pose.position.x = 7.88904; // Adjust based on the workspace
        //pickup_table_pose.position.y = -2.99049; // Adjust based on the workspace
        pickup_table_pose.position.x = 0.75; // Adjust based on the workspace
        pickup_table_pose.position.y = -0.15; // Adjust based on the workspace
        pickup_table_pose.position.z = 0.375; // Half the height of the table for the center point

        // Assign primitive and pose to the collision object
        pickup_table.primitives.push_back(pickup_table_primitive);
        pickup_table.primitive_poses.push_back(pickup_table_pose);
        pickup_table.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(pickup_table);
        // apply all the collision objects
		ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));
		bool success = planning_scene.applyCollisionObjects(collision_objects);
		if(success) ROS_INFO("Successfully added the collision objects");
		else ROS_WARN("Failed to add the collision objects");
    }   

    geometry_msgs::Pose computePlacingPose(int i)
    {
        // set the x coordinates to place the object
        ROS_INFO("Coordinates of origin of table: x: %f, y:%f",line_origin.pose.position.x,line_origin.pose.position.y);
        double x_coordinate = (y_coordinates[i] - q) / m;
        geometry_msgs::Pose placing_pose = line_origin.pose;
        placing_pose.position.y -= x_coordinate;
        placing_pose.position.x += y_coordinates[i] + q;
        ROS_INFO("Coordinates of placing pose x: %f, y:%f",placing_pose.position.x,placing_pose.position.y);

        return placing_pose;
    }

    void reach(geometry_msgs::Pose& pose) {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();

        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;

        // Add 0.20m offset along the z-axis for approaching the object
        target_pose.position.z -= 0.20;

        ROS_INFO("Planning Cartesian path to touch the object...");
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose);        // Starting position
        waypoints.push_back(target_pose); // Target position

        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Step size for the end-effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        ROS_INFO("Cartesian path planning reached %.2f%% of its trajectory", fraction * 100);

        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;

        ROS_INFO("Executing Cartesian path...");
        auto execution_result = move_group.execute(cartesian_plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Initial configuration successfully reached!");

        // Stop the spinner after the function is complete
        spinner.stop();
    }

    void detach(int32_t id) {

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
        ROS_INFO("Model name to detach: %s",srv.request.model_name_2.c_str());
        ROS_INFO("Link name to detach: %s",srv.request.link_name_2.c_str());
        if (attach_client.call(srv)) {
            ROS_INFO("Successfully detached object to the gripper.");
            return;
        } else {
            ROS_ERROR("Failed to detached object to the gripper.");
            return;
        }
    }

    void openGripper(){

        ROS_INFO("Opening the gripper...");
        // Create the trajectory goal
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("gripper_joint"); // Replace with your gripper joint name

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(1.0); // Fully closed position
        point.time_from_start = ros::Duration(1.0); // Adjust time for motion

        goal.trajectory.points.push_back(point);

        ROS_INFO("Sending gripper open trajectory goal...");
        gripper_client.sendGoal(goal);

        bool success = gripper_client.waitForResult(ros::Duration(5.0));
        if (success) {
            ROS_INFO("Gripper opened successfully.");
        } else {
            ROS_ERROR("Failed to open the gripper.");
        }
    }
    
    void depart(geometry_msgs::Pose& pose ){

        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();                
        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;

        // Add 0.25m offset along the z-axis
        target_pose.position.z += 0.20;

        ROS_INFO("Planning Cartesian path to depart ...");
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose);
        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01; // Step size for end-effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        ROS_INFO("Cartesian path planning reached %.2f%% of its trajectory", fraction * 100);

        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;

        ROS_INFO("Executing Cartesian path...");
        auto execution_result = move_group.execute(cartesian_plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Initial configuration successfully reached!");
        // Stop the spinner after finishing
        spinner.stop();
    }

    void tuck_config(){
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();  
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

        move_group.setPlanningTime(15.0);
        move_group.setNumPlanningAttempts(5);
        ROS_INFO("Setting Tuck arm configuration for Tiago's arm...");
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }
        ROS_INFO("Motion plan successfully generated!");

        ROS_INFO("Executing the motion...");
        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Tuck Arm configuration set successfully.");
        // Stop the spinner after finishing
        spinner.stop();
    }

    void clearMoveItObjects()
    {
        // Clear the planning scene
        std::vector<std::string> object_ids = planning_scene.getKnownObjectNames();
        if (!object_ids.empty()) {
            planning_scene.removeCollisionObjects(object_ids);
            ROS_INFO("Removed %zu collision objects from the planning scene.", object_ids.size());
        } else {
            ROS_INFO("No collision objects to remove.");
        }
    }

    void Placing(int i)
    {
        
        CollisionTable();
        position.push_back({0.0, -M_PI / 3.0}); // Look left 25 degrees (maintaining down)
        moveHead(position); 
        ros::Duration(1.0).sleep();
        position.clear();
        // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
        object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeA::AprilTagDetectionCallback, this);
        ros::spinOnce();
        ros::Duration(1.0).sleep();

        // here the head is pointing in the left low corner of the table to detect the apriltag
            // Main loop
        while (ros::ok()) {
            // Process incoming messages
            ros::spinOnce();
            if(activated == true) break;
            // Other logic if needed
            // e.g., controlling the robot or checking other conditions
            ros::Duration(0.1).sleep(); // Sleep for 100ms to avoid busy looping
        }
        ROS_INFO("Terminated Detection");
        activated = false; // reset for next iteration
        // Shutdown subscriber to avoid future callbacks
        object_detection_sub.shutdown();
        ROS_INFO("Object detection subscriber shut down.");
        // change the end effector frame to perform the picking operation
        initial_config();
        // Create the pose to place the object based on the line equation
        geometry_msgs::Pose placing_pose = computePlacingPose(i);  
        approach(placing_pose);
        ros::Duration(1.0).sleep();
        reach(placing_pose);
        openGripper();
        ros::Duration(1.5).sleep();
        detach(id);
        ros::Duration(1.0).sleep();
        depart(placing_pose);
        initial_config();
        tuck_config();

        ROS_INFO("Tuck arm configuration completed.");
       // Clear collision objects safely
        if (!collision_objects.empty()) {
            collision_objects.clear();
            ROS_INFO("Collision objects cleared.");
        } else {
            ROS_WARN("Collision objects already empty.");
        }
        clearMoveItObjects();
        line_origin.header.frame_id = "";  // Set to an empty string to clear it
        if(line_origin.header.frame_id.empty()) ROS_INFO("line_origin successfully cleared");
    }
    std::vector<moveit_msgs::CollisionObject> collision_objects; // vector containing all collision objects detected


private:

    // CLASS VARIABLES
    ros::NodeHandle& nh_;
    ros::ServiceClient client_; // Service client for straight_line_srv
    MoveBaseClient move_base_client_; // Action client for move_base
	ros::ServiceClient picking_client_; // Service client to picking server
    double m; // Straight line m parameter
    double q; // Straight line q parameter
    std::vector<move_base_msgs::MoveBaseGoal> routineA; // first routine to get tiago from initial pose to picking pose
    std::vector<move_base_msgs::MoveBaseGoal> routineB; // second routine to move tiago from picking pose to placing pose and viceversa
	ros::Publisher tilt_cam_pub; // publisher for the initial tilt of the camera
	ros::Publisher detection_pub; // Publisher to send commands to NodeB
	ros::Subscriber picking_sub;   // Subscriber to listen to status updates from NodeB
    ros::Subscriber object_detection_sub; // subscriber for apriltag detection
    TrajectoryClient head_client;
    TrajectoryClient torso_client_;
    geometry_msgs::PoseStamped line_origin;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
	int id; // id of the picked object
    ros::ServiceClient attach_client;
    tf2_ros::Buffer tf_buffer; 
    tf2_ros::TransformListener tf_listener;
    TrajectoryClient gripper_client;
    std::vector<double> y_coordinates = {0.0,0.1,0.2};
    // Initialize MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<std::pair<double, double>> position;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Node_A");
    ros::NodeHandle nh;

    NodeA nodeA(nh); // Node_A constructor
	// Get m and q
	nodeA.callLineService();

	// Navigate to the Picking Pose
	nodeA.navigateToPickingPose();
    nodeA.liftTorso();

    // START THE LOOP
    for(size_t i = 0; i < 3; i++)
    {
        ROS_INFO("Iteration number: %ld, before initializeDetection to check segmentation fault", i);
	    nodeA.initializeDetection();
        ros::Rate rate(1);
        while(ros::ok() && !nodeA.picked_object) 
        {
            ros::spinOnce(); // Process callbacks
            rate.sleep();
        }   
        nodeA.picked_object = false; // reset for next iteration 
        nodeA.navigateToPlacingPose();
        //ros::spinOnce(); // Process callbacks
        ros::Duration(1.0).sleep();
        nodeA.Placing(i);
        ROS_INFO("Go back to the picking pose!");
        nodeA.navigateBackToPickingPose();

    }
    // send goal to Node_B to detect a pickable object and pick it
    // send goal to Node_C to pick that object and 
    // manipulate it for transportation

    // Navigate to the Placing Pose

    // detect AprilTag ID=10 for the placing frame

    // Placing operation

    // --------------------------------------------
    // (repeat this 3 times)
    ros::waitForShutdown();
    return 0;
}
