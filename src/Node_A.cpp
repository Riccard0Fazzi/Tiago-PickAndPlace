#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> 
#include <move_base_msgs/MoveBaseAction.h> // to navigate Tiago
#include <tiago_iaslab_simulation/Coeffs.h> // to request m,q from /straight_line_srv
#include <ir2425_group_24_a2/picking.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Alias for the move_base action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NodeA
{
public:

    // CONSTRUCTOR
    NodeA(ros::NodeHandle& nh)
        : client_(nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv")), // service client to the straight line service
          move_base_client_("/move_base", true), // action client to the move_base action server
		  picking_client_(nh.serviceClient<ir2425_group_24_a2::picking>("picking")) // service client to the Node_B server
    {
        ROS_INFO("Node A initialized and ready to call /straight_line_srv service.");

        // Wait for the move_base action server to start
        ROS_INFO("Waiting for the /move_base action server to start...");
        move_base_client_.waitForServer();
        ROS_INFO("/move_base action server started.");
		// publisher for the initial tilt of the camera 
		// to be ready to detect aprilTags
		tilt_cam_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
		// set up the two routines 
		initialize_routines(); 
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

    void TiltCamera(){
		// define rate for the initialization operation
		ros::Rate cam_init_r(1);
		// waiting for subscribers to /head_controller/command
		while(tilt_cam_pub.getNumSubscribers() == 0 && ros::ok()){
			cam_init_r.sleep();
		}
		// initialize object containing the command to
		// send to Tiago
		trajectory_msgs::JointTrajectory tilt_cmd;
		trajectory_msgs::JointTrajectoryPoint point;
		// set the tilt command
		tilt_cmd.joint_names = {"head_1_joint","head_2_joint"};
		point.positions = {0.0,-1.0};
		point.time_from_start = ros::Duration(1.0);
		tilt_cmd.points.push_back(point);
		// send the tilt command to Tiago
		tilt_cam_pub.publish(tilt_cmd);
		// wait for Tiago to incline the camera ..
		cam_init_r.sleep();
		// publish feedback
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

	// PICKING method
    // __________________________________________________
    // method to request the /picking server to detect
	// a pickable object and then start the picking action
    // in Node_C server, once the picking action is completed
	// the method terminates
    void Picking()
    {
        // Create a service request and response object
		ir2425_group_24_a2::picking srv;

        // Set the 'activate_detection' field in the request
        srv.request.activate_detection = true;

        // Call the service
        if (picking_client_.call(srv))
        {
            ROS_INFO("/picking service called successfully.");
            ROS_INFO("Picked object ID = %d",srv.response.picked_obj_id);
        }
        else
        {
            ROS_ERROR("Failed to call /picking service");
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

    void navigateRoutineB()
    {
		for (size_t i = 0; i < routineB.size(); ++i) {

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
	// Tilt Camera procedure
	//nodeA.TiltCamera();

    // send goal to Node_B to detect a pickable object and pick it
	nodeA.Picking();
    // send goal to Node_C to pick that object and 
    // manipulate it for transportation

    // Navigate to the Placing Pose

    // detect AprilTag ID=10 for the placing frame

    // Placing operation

    // --------------------------------------------
    // (repeat this 3 times)

    return 0;
}
