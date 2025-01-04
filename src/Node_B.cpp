#include <ros/ros.h>
#include <ir2425_group_24_a2/picking.h>

class NodeB
{
	public:
		NodeB() {
			// Initialize the service server
			service_ = nh_.advertiseService("picking", &NodeB::DetectionCallback, this);
			ROS_INFO("Node_B server is ready to receive requests.");
		}

	private:
		// Node handle
		ros::NodeHandle nh_;

		// Service server
		ros::ServiceServer service_;

		// Callback for the service
		bool DetectionCallback(ir2425_group_24_a2::picking::Request &req, ir2425_group_24_a2::picking::Response &res) {
			ROS_INFO("Received request from Node_A!");

			// Process the request and populate the response
			res.picked_obj_id = 3; 
			ROS_INFO("Object ID = %d picking action completed", res.picked_obj_id);
			return true;
		}
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "Node_B");

    // Create an instance of the server class
    NodeB node_b_server;

    // Spin to process incoming requests
    ros::spin();

    return 0;
}
