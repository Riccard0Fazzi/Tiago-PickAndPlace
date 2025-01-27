#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> 
#include <move_base_msgs/MoveBaseAction.h> 
#include <tiago_iaslab_simulation/Coeffs.h> 
#include <ir2425_group_24_a2/detection.h>
#include <ir2425_group_24_a2/picking_completed.h>
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

    // CLASS public VARIABLES
    bool picked_object; // true when the object is picked 
    bool activated; // true when the detection should be activated

    // [CONSTRUCTOR]
    NodeA(ros::NodeHandle& nh)
        : nh_(nh),
          client_(nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv")),                 // service client to the straight line service
          move_base_client_("/move_base", true),                                                            // action client to move tiago
          head_client("/head_controller/follow_joint_trajectory", true),                                    // action client to move tiago's head
          torso_client_("/torso_controller/follow_joint_trajectory", true),                                 // action client to move tiago's torso
          tf_listener(tf_buffer),                                                                           // buffer for tf transform
          attach_client(nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach")),  // service client to detach objects
          gripper_client("/parallel_gripper_controller/follow_joint_trajectory", true),                     // action client to handle the gripper
          move_group("arm")

    {
        // waiting for the servers
        move_base_client_.waitForServer();
        head_client.waitForServer();
        torso_client_.waitForServer();
        gripper_client.waitForServer();
		tilt_cam_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10); // to publish controls to move the head
		detection_pub = nh.advertise<ir2425_group_24_a2::detection>("/start_detection", 10); // to publish the message to Node_B to start detecting
		picking_sub = nh.subscribe("/picking_terminated", 10, &NodeA::PickingTerminatedCallBack, this); // to receive the message of termination of the picking  
        move_group.setEndEffectorLink("gripper_base_link"); // set the frame for the e-e
		initialize_routines(); // construct the vector of waypoints for the  overall navigation
        move_group.setPlanningTime(15.0); // give enough time for the moveIt planner
        move_group.setNumPlanningAttempts(5); // give enough attempts for the moveIt planner
        picked_object = false; // initialize boolean, true when the object is picked          
        activated = false; // initialize boolean, true when the detection should be activated
    }

    // [CALLBACK] to receive the message to terminate the detection 
    // ______________________________________________________________________________________
	void PickingTerminatedCallBack(const ir2425_group_24_a2::picking_completed::ConstPtr& msg)
	{
		if(msg->picking_completed)
		{
			ROS_INFO("Picking completed!");
			ROS_INFO("Starting placing procedure");
            picked_object = true; // update the boolean
            id = msg->id;	// saving the ID of the object that will be used to do the detach		
            return;
		}
	}

    // [CALLBACK] for AprilTag Detection
    // ______________________________________________
    // retrieves the pose of the table frame to place 
    // the object along the straight line
    void AprilTagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) 
    {
        if(msg->detections.empty()) return;
        // the callback just check once if the table frame AprilTag was detected
        for(const auto& detection : msg->detections)
        {
            int id_tag = detection.id[0];
            if(id_tag==10 && line_origin.header.frame_id.empty()) // check if table frame
            {
                // get the pose of the tag
                geometry_msgs::PoseStamped frame;
                frame.header.seq = static_cast<uint32_t>(id_tag); // storing ID
                frame.header.frame_id = detection.pose.header.frame_id; // Get frame_id from the detection
                frame.pose.position = detection.pose.pose.pose.position; // saving position
                frame.pose.orientation = detection.pose.pose.pose.orientation;// saving orientation
                // Define a transform from the camera frame (or detected frame) to the map frame
                ros::Rate rate(100.0);  // Loop frequency in Hz
                // transform from camera frame to map frame
                while (ros::ok()) 
                {
                    if(tf_buffer.canTransform("base_footprint", frame.header.frame_id, ros::Time::now(), ros::Duration(1.0))) {
                        try {
                            tf_buffer.transform(frame, line_origin, "base_footprint", ros::Duration(0.1)); // store the pose in line_origin
                            break;  // Exit loop after successful transformation
                        } catch (tf2::TransformException &ex) {
                            ROS_WARN("Could not transform pose from %s to base_footprint frame: %s", frame.header.frame_id.c_str(), ex.what());
                        }
                    }
                    rate.sleep();    
                }	
                activated = true; // don't process again this callaback
                ros::spinOnce();
                break;
            }
        }
	}

    // [METHOD] to initialize the routines
    // ______________________________________________
    // to initialize the vectors containing the goals
    // to give to move_base for the navigation
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
 
		// first WAY-CORNER
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

        // ROUTINE B 
        // ________________________________________________________
        // (from picking pose to placing pose and then to out 
        // of corridor pose)

		// PICKING POSE orientation
		goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -3.71049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 1.0; // cos(π/4)
		
		routineB.push_back(goal);

        // second WAY-CORNER
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

        // BACK-UP from Placing Pose
        goal.target_pose.pose.position.x = 8.8; // x-coordinate
        goal.target_pose.pose.position.y = -1.5; // y-coordinate
        goal.target_pose.pose.orientation.z =  0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)

		routineB.push_back(goal);

        // third WAY-CORNER
        goal.target_pose.pose.position.x = 8.8; // x-coordinate
        goal.target_pose.pose.position.y = 0.0; // y-coordinate
        goal.target_pose.pose.orientation.z = 1.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.0; // cos(π/4)

		routineB.push_back(goal);
    }

    // [METHOD] to initialize and support the detection
    // ________________________________________________
    // this method moves Tiago's head and in resting
    // poses it publish a message in Node_B to perform
    // the detection
    void initializeDetection() {
        ir2425_group_24_a2::detection msg;
        // Define pan and tilt points
        std::vector<std::pair<double, double>> positions = {
            {0.0, -M_PI / 4.0},             // Look down 45 degrees
            {+M_PI / 7.2, -M_PI / 4.0},     // Look left 25 degrees (maintaining down)
            {-M_PI / 7.2, -M_PI / 4.0},     // Look right 50 degrees (from the previous position)
            {0.0, -M_PI / 4.0}              // return to initial pose 
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
            
            // Monitor the result 
            while (!head_client.waitForResult(ros::Duration(0.1))) {
                ros::spinOnce();  
            }
            // Check if the goal was successfully executed
            if (head_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_WARN("Failed to move camera to position: pan=%f, tilt=%f", pos.first, pos.second);
            }
            // Wait for one second before moving to the next position
            ros::Duration(1.0).sleep();
            // Publish the message to initialize detection
            msg.collect = true;
            detection_pub.publish(msg);
            ros::spinOnce();
            // wait for the transform (in the resting pose) to be available before moving again the head
            while (ros::ok()) {
                if(tf_buffer.canTransform("base_footprint", "xtion_rgb_frame", ros::Time::now(), ros::Duration(1.0))) break;
            }
            ros::Duration(1.0).sleep();
        }
        // publish the message to collect all the detected objects before terminating the detection
        msg.collect = false;
        // Publish detection message 
        detection_pub.publish(msg);
        ros::spinOnce();
        ros::Duration(1.0).sleep();

    }

    // [METHOD] to generally move Tiago's head
    // ________________________________________________
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
               ros::spinOnce(); 
            }

            if (head_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_WARN("Failed to move camera to position: pan=%f, tilt=%f", pos.first, pos.second);
            }
        }
    }

    // [METHOD] to retrive the m and q coefficients
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

    // [METHOD] PICKING POSE NAVIGATION 
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
    
    // [METHOD] PLACING POSE NAVIGATION
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
    
    // [METHOD] BACK TO PICKING POSE NAVIGATION 
    // __________________________________________________
    // method to navigate Tiago back to the picking pose 
    // from the placing pose
    void navigateBackToPickingPose()
    {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();
        routineB[3].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
		// Navigation to the Picking Pose
		// Send the goal to move_base
		move_base_client_.sendGoal(routineB[3]);
		// wait for the result
		move_base_client_.waitForResult();
        // wait for stable routine
        ros::Duration(0.5).sleep();
        routineB[4].target_pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
		// Send the goal to move_base
		move_base_client_.sendGoal(routineB[4]);
		// wait for the result
		move_base_client_.waitForResult();
        // wait for stable routine
        ros::Duration(0.5).sleep();
        navigateToPickingPose(); // re-use previous method
		if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("The robot reached the Picking Pose again successfully.");
		else
			ROS_WARN("The robot failed to reach the Picking Pose.");
        // Stop the spinner after finishing
        spinner.stop();
    }

    // [METHOD] lifting up Tiago's torso for better detection
    // ______________________________________________________
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
        torso_client_.sendGoal(goal);
        // Wait for the result
        torso_client_.waitForResult();
        if (torso_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Successfully lifted torso to maximum extension.");
        } else {
            ROS_ERROR("Failed to lift torso.");
        }
    }

    // [METHOD] sets the initial configuration for Tiago's arm
    // ________________________________________________________
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
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }

        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Initial configuration successfully reached!");

        // Stop the spinner after finishing
        spinner.stop();
    }
    
    // [METHOD] to approach the object to pick
    // _______________________________________________________
    void approach(geometry_msgs::Pose& pose) {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();
        // Adjust the target pose to be 25 cm above the given pose
        pose.position.z += 0.25;
        // Extract the current orientation from the pose
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
        // Check if out of workspace
        bool is_within_bounds = move_group.setPoseTarget(pose);
        if (!is_within_bounds) {
            ROS_ERROR("Target pose is outside the robot's workspace.");
            return;
        }
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }
        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Motion to approach pose completed.");
        // Stop the spinner after finishing
        spinner.stop();
    }


    // [METHOD] adds the table as a collision object
    // ___________________________________________________________
    void CollisionTable()
    {
        // Table dimensions (slightly larger than real one for safety)
        double table_size = 0.95; 
        // Pick-up table
        moveit_msgs::CollisionObject pickup_table;
        pickup_table.header.frame_id = "base_footprint";  
        pickup_table.id = "pickup_table";
        // Define cube dimensions
        shape_msgs::SolidPrimitive pickup_table_primitive;
        pickup_table_primitive.type = shape_msgs::SolidPrimitive::BOX;
        pickup_table_primitive.dimensions.resize(3);
        pickup_table_primitive.dimensions[0] = table_size; // Length
        pickup_table_primitive.dimensions[1] = table_size; // Width
        pickup_table_primitive.dimensions[2] = 0.78; // Height
        // Define the positions
        geometry_msgs::Pose pickup_table_pose;
        pickup_table_pose.position.x = 0.75; 
        pickup_table_pose.position.y = -0.15; 
        pickup_table_pose.position.z = 0.375; // Half the height of the table for the center point
        // Assign primitive and pose to the collision object
        pickup_table.primitives.push_back(pickup_table_primitive);
        pickup_table.primitive_poses.push_back(pickup_table_pose);
        pickup_table.operation = moveit_msgs::CollisionObject::ADD;
        collision_objects.push_back(pickup_table);
        // apply all the collision objects (just the table)
		ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));
		bool success = planning_scene.applyCollisionObjects(collision_objects);
		if(success) ROS_INFO("Successfully added the collision objects");
		else ROS_WARN("Failed to add the collision objects");
    }   

    // [METHOD] to compute the pose for the placing
    // ________________________________________________________
    // following the m,q for the line service
    geometry_msgs::Pose computePlacingPose(int i)
    {
        geometry_msgs::Pose placing_pose;
        if(m<1){ // start with the x coordinates and compute the y of the line
            double y_coordinate = m * coordinates[i] + q;
            placing_pose = line_origin.pose;
            placing_pose.position.y -= coordinates[i];
            placing_pose.position.x += y_coordinate;
        }
        else{ // start with the y and compute the x of the line
            double x_coordinate = coordinates[i]/m;
            placing_pose = line_origin.pose;
            placing_pose.position.y -= x_coordinate;
            placing_pose.position.x += coordinates[i] + q;
        }
        return placing_pose;
    }

    // [METHOD] to perform the linear movement to reach the object
    // ___________________________________________________________
    void reach(geometry_msgs::Pose& pose) {
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();
        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;
        target_pose.position.z -= 0.20;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(pose);        // Starting position
        waypoints.push_back(target_pose); // Target position
        moveit_msgs::RobotTrajectory trajectory;
        // generate the linear movement until there's the object
        const double eef_step = 0.01; // Step size for the end-effector
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
        // ROS_INFO("Cartesian path planning reached %.2f%% of its trajectory", fraction * 100);
        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        auto execution_result = move_group.execute(cartesian_plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Reaching succesfully performed!");
        // Stop the spinner after the function is complete
        spinner.stop();
    }

    // [METHOD] to detach the object form the e-e link
    // ___________________________________________________________
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
        if (attach_client.call(srv)) {
            ROS_INFO("Successfully detached object to the gripper.");
            return;
        } else {
            ROS_ERROR("Failed to detached object to the gripper.");
            return;
        }
    }

    // [METHOD] to open the gripper
    // ___________________________________________________________
    void openGripper(){
        // Create the trajectory goal
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("gripper_joint"); // Replace with your gripper joint name
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(1.0); // Fully closed position
        point.time_from_start = ros::Duration(1.0); // Adjust time for motion
        goal.trajectory.points.push_back(point);
        gripper_client.sendGoal(goal);
        bool success = gripper_client.waitForResult(ros::Duration(5.0));
        if (success) {
            ROS_INFO("Gripper opened successfully.");
        } else {
            ROS_ERROR("Failed to open the gripper.");
        }
    }
    
    // [METHOD] to depart from the placed object
    // ___________________________________________________________
    void depart(geometry_msgs::Pose& pose ){
        // Start an AsyncSpinner to handle callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();                
        // Perform linear movement to grasp the object
        geometry_msgs::Pose target_pose = pose;
        // Add 0.20m offset along the z-axis
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
        auto execution_result = move_group.execute(cartesian_plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Departing performed succesfully!");
        // Stop the spinner after finishing
        spinner.stop();
    }

    // [METHOD] to set the arm to a safe configuration for moving
    // ___________________________________________________________
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
        auto planning_result = move_group.plan(plan);
        if(planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion planning failed!");
            return;
        }
        auto execution_result = move_group.execute(plan);
        if(execution_result != moveit::planning_interface::MoveItErrorCode::SUCCESS){
            ROS_ERROR("Motion execution failed!");
            return;
        }
        ROS_INFO("Tuck configuration set successfully.");
        // Stop the spinner after finishing
        spinner.stop();
    }

    // [METHOD] to clear the planning scene for future placings
    // ___________________________________________________________
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

    // [METHOD] that globally handles the placing operation
    // ___________________________________________________________
    void Placing(int i)
    {
        CollisionTable(); // add table as a collision object
        position.push_back({0.0, -M_PI / 3.0}); // Look left 25 degrees (maintaining down) to see the table frame
        moveHead(position); 
        ros::Duration(1.0).sleep();
        position.clear();
        // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
        object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeA::AprilTagDetectionCallback, this);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        // here the head is pointing in the left low corner of the table to detect the apriltag
        // wait for the detection to be terminated
        while (ros::ok()) {
            // Process incoming messages
            ros::spinOnce();
            if(activated == true) break;
            // Other logic if needed
            // e.g., controlling the robot or checking other conditions
            ros::Duration(0.1).sleep(); // Sleep for 100ms to avoid busy looping
        }
        activated = false; // reset for next iteration
        // Shutdown subscriber to avoid future callbacks
        object_detection_sub.shutdown();
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
        ROS_INFO("Placing terminated!");
       // Clear collision objects safely
        if (!collision_objects.empty()) {
            collision_objects.clear();
        } 
        clearMoveItObjects();
        line_origin.header.frame_id = "";  // Set to an empty string to clear it
    }


private:

    // CLASS private VARIABLES
    ros::NodeHandle& nh_;
    ros::ServiceClient client_; // Service client for straight_line_srv
    MoveBaseClient move_base_client_; // Action client for move_base
	ros::ServiceClient picking_client_; // Service client to picking server
    double m; // Straight line m parameter
    double q; // Straight line q parameter
    std::vector<move_base_msgs::MoveBaseGoal> routineA; // first routine to get tiago from initial pose to picking pose
    std::vector<move_base_msgs::MoveBaseGoal> routineB; // second routine to move tiago from picking pose to placing pose and go back
	ros::Publisher tilt_cam_pub; // publisher for the initial tilt of the camera
	ros::Publisher detection_pub; // Publisher to send commands to NodeB
	ros::Subscriber picking_sub;   // Subscriber to listen to status updates from NodeB
    ros::Subscriber object_detection_sub; // subscriber for apriltag detection
    TrajectoryClient head_client; // client for head movement
    TrajectoryClient torso_client_; // client for torso movement
    geometry_msgs::PoseStamped line_origin; // object to store the table frame pose
	int id; // id of the picked object
    ros::ServiceClient attach_client; // client for ros_link_attacher
    tf2_ros::Buffer tf_buffer; // buffer for the tf transform
    tf2_ros::TransformListener tf_listener; // listener for the tf transform
    TrajectoryClient gripper_client; // client to close the gripper
    std::vector<double> coordinates = {0.0,0.15,0.30}; // coordinates chosen for the placing line
    // Initialize MoveIt interfaces
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<std::pair<double, double>> position; // stores pan and tilt pos to move the head
    std::vector<moveit_msgs::CollisionObject> collision_objects; // vector containing all collision objects detected

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
    // START THE LOOP: handles picking and placing operations
    for(size_t i = 0; i < 3; i++)
    {
	    nodeA.initializeDetection();
        ros::Rate rate(1);
        while(ros::ok() && !nodeA.picked_object) 
        {
            ros::spinOnce(); // Process callbacks
            rate.sleep();
        }   
        nodeA.picked_object = false; // reset for next iteration 
        nodeA.navigateToPlacingPose();
        ros::Duration(1.0).sleep();
        nodeA.Placing(i);
        nodeA.navigateBackToPickingPose();
    }
    ros::waitForShutdown();
    return 0;
}
