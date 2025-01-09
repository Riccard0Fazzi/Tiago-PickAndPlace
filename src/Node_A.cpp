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

// Alias for the move_base action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class NodeA
{
public:

    // CONSTRUCTOR
    NodeA(ros::NodeHandle& nh)
        : client_(nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv")), // service client to the straight line service
          move_base_client_("/move_base", true), // action client to the move_base action server
          head_client("/head_controller/follow_joint_trajectory", true)
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
		// set up the two routines 
		initialize_routines(); 
    }

	void PickingTerminatedCallBack(const ir2425_group_24_a2::picking_completed::ConstPtr& msg)
	{
		if(msg->picking_completed)
		{
			ROS_INFO("Picking completed!");
			ROS_INFO("Starting placing procedure");
			return;
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
        goal.target_pose.pose.orientation.z = 0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
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
        goal.target_pose.pose.position.x = 8.53904; // x-coordinate
        goal.target_pose.pose.position.y = -1.85049; // y-coordinate
        goal.target_pose.pose.orientation.z = 1.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.0; // cos(π/4)

		routineB.push_back(goal);

        // POST PLACING ORIENTATION
        goal.target_pose.pose.position.x = 8.53904;; // x-coordinate
        goal.target_pose.pose.position.y = -1.85049; // y-coordinate
        goal.target_pose.pose.orientation.z = -0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
		routineB.push_back(goal);

        // upper WAY-CORNER
        goal.target_pose.pose.position.x = 8.83904; // x-coordinate
        goal.target_pose.pose.position.y = -4.01049; // y-coordinate
        goal.target_pose.pose.orientation.z = 1.0; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.0; // cos(π/4)
		
		routineB.push_back(goal);

        // PICKING POSE
        goal.target_pose.pose.position.x = 7.83904; // x-coordinate
        goal.target_pose.pose.position.y = -3.71049; // y-coordinate
        goal.target_pose.pose.orientation.z = 0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)
		
		routineB.push_back(goal);


   }

    void Detection() {
            ir2425_group_24_a2::detection msg;
            msg.activate_detection = true;
            msg.start_picking = false;
            // Tilt Camera procedure
            detection_pub.publish(msg);
            ros::spinOnce();
            ros::Duration(2.0).sleep();

            // Define pan and tilt points
            std::vector<std::pair<double, double>> positions = {
                {0.0, -M_PI / 4.0},  // Initial position (center, 45 degrees down)
                {-0.3, -M_PI / 4.0 + 0.2},  // Slightly left
                {0.3, -M_PI / 4.0 + 0.2},  // Slightly right
                {0.0, -M_PI / 4.0}         // Return to center
            };

            // Iterate through the positions
            for (const auto& pos : positions) {
                // Create a FollowJointTrajectoryGoal message
                control_msgs::FollowJointTrajectoryGoal goal;
                goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

                // Create a trajectory point
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = {pos.first, pos.second};  // Set pan and tilt
                point.time_from_start = ros::Duration(2.0);  // 1 second to reach the position
                goal.trajectory.points.push_back(point);

                // Send the goal
                head_client.sendGoal(goal);

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
            }

            ROS_INFO("Camera movement routine completed.");
            msg.activate_detection = false;
            msg.start_picking = true;
            // Tilt Camera procedure
            detection_pub.publish(msg);
            ros::spinOnce();
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

private:

    // CLASS VARIABLES
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
    TrajectoryClient head_client;
	
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

	nodeA.Detection();
    // send goal to Node_B to detect a pickable object and pick it
    // send goal to Node_C to pick that object and 
    // manipulate it for transportation

    // Navigate to the Placing Pose

    // detect AprilTag ID=10 for the placing frame

    // Placing operation

    // --------------------------------------------
    // (repeat this 3 times)

    return 0;
}
