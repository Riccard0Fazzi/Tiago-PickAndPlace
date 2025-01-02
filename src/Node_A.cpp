#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> // WASSUPPPP
#include <move_base_msgs/MoveBaseAction.h> // to navigate Tiago
#include <tiago_iaslab_simulation/Coeffs.h> // to request m,q from /straight_line_srv

// Alias for the move_base action client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NodeA
{
public:

    // CONSTRUCTOR
    NodeA(ros::NodeHandle& nh)
        : client_(nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv")), // service client to the straight line service
          move_base_client_("/move_base", true) // action client to the move_base action server
    {
        ROS_INFO("Node A initialized and ready to call /straight_line_srv service.");
        // Wait for the move_base action server to start
        ROS_INFO("Waiting for the move_base action server to start...");
        move_base_client_.waitForServer();
        ROS_INFO("move_base action server started.");
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
            ROS_ERROR("Failed to call service /straight_line_srv service");
        }
    }

    // PICKING POSE NAVIGATION method
    // __________________________________________________
    // method to navigate Tiago to the picking pose to be
    // able to do the picking task
    void navigateToPickingPose()
    {
        // Define the Picking Pose 
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map"; // Use the map frame
        goal.target_pose.header.stamp = ros::Time::now();

        // Hard-coded Picking Pose
        // position
        goal.target_pose.pose.position.x = 7.96503; // x-coordinate
        goal.target_pose.pose.position.y = -3.74143; // y-coordinate
        goal.target_pose.pose.position.z = 0.0; // z-coordinate
        // orientation
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.7071; // sin(π/4)
        goal.target_pose.pose.orientation.w = 0.7071; // cos(π/4)

        // Navigation to the Picking Pose
        ROS_INFO("Navigate to the Picking Pose: x = %f, y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        // Send the goal to move_base
        move_base_client_.sendGoal(goal);

        // Wait for the result
        bool finished_before_timeout = move_base_client_.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
            if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("The robot reached the Picking Pose successfully.");
            }
            else
            {
                ROS_WARN("The robot failed to reach the Picking Pose.");
            }
        }
        else
        {
            ROS_ERROR("Timed out waiting for the robot to reach the Picking Pose.");
        }
    }

private:

    // CLASS VARIABLES
    ros::ServiceClient client_; // Service client for straight_line_srv
    MoveBaseClient move_base_client_; // Action client for move_base
    double m; // Straight line m parameter
    double q; // Straight line q parameter
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

    // send goal to Node_B to detect a pickable object
    // receiving the pose of the object
    
    // send goal to Node_C to pick that object and 
    // manipulate it for transportation

    // Navigate to the Placing Pose

    // detect AprilTag ID=10 for the placing frame

    // Placing operation

    // --------------------------------------------
    // (repeat this 3 times)

    return 0;
}
