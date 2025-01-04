#include <ros/ros.h>
#include <ir2425_group_24_a2/picking.h>
// import for tiago eyes callback
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// import for ObjectDetectionCallback
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdint>

class NodeB
{
	public:

		NodeB() {
			// Initialize the service server
			service_ = nh_.advertiseService("picking", &NodeB::DetectionCallback, this);
			ROS_INFO("Node_B server is ready to receive requests.");
			// Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_); // image transport for the camera topic
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &NodeB::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeB::ObjectDetectionCallback, this);
		}

	private:

		// Node handle
		ros::NodeHandle nh_;
		// Service server
		ros::ServiceServer service_;
		// vector for storing the detected objects frames
		std::vector<geometry_msgs::PoseStamped> detected_objects;
		ros::Subscriber object_detection_sub; // subscriber for apriltag detection
		image_transport::Subscriber image_sub; // subscriber for camera
		bool activated; // variable to trigger the detection of the objects

		// Callback for the service
		bool DetectionCallback(ir2425_group_24_a2::picking::Request &req, ir2425_group_24_a2::picking::Response &res) {
			ROS_INFO("Received request from Node_A!");
			activated = req.activate_detection;
			// Process the request and populate the response
			res.picked_obj_id = 3; 
			ROS_INFO("Object ID = %d picking action completed", res.picked_obj_id);
			ros::Duration(2.0).sleep();
			activated = false;

			return true;
		}
		
		// CallBack to display Tiago's view
        // ______________________________________________
        // [VISUAL PURPOSE ONLY]
        // show the current RGB image (30Hz)
        void tiagoEyesCallback(const sensor_msgs::ImageConstPtr& msg) {
            try {
                // create the mat object to store Tiago's camera current image
                cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
                // let's downscale the image for visual purposes
                int down_width = img.cols/2;
                int down_height = img.rows/2;
                cv::Mat resized_down;
                //resize down
                resize(img, resized_down, cv::Size(down_width, down_height), cv::INTER_LINEAR);
                        // Display the image
                        cv::imshow("Tiago Eyes", resized_down);
                        cv::waitKey(1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
            }
        }

		// CallBack for AprilTags Detection
        // ______________________________________________
        // Callback that continuously search for AprilTags
        // in Tiago's camera and compute their position
        // wrt map frame (20 Hz)
        void ObjectDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {

            // callBack always running, saving detected poses only when required
            if (activated) {
				detected_objects.clear();
				geometry_msgs::PoseStamped frame;
				frame.header.seq = static_cast<uint32_t>(msg->detections[0].id[0]); // saving ID
				ROS_INFO("Detected object ID: %u",frame.header.seq);
				frame.pose.position = msg->detections[0].pose.pose.pose.position; // saving position
				frame.pose.orientation = msg->detections[0].pose.pose.pose.orientation;// saving orientation

				detected_objects.push_back(frame);	
            }
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
