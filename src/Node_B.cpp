#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ir2425_group_24_a2/picking_completed.h>
#include <ir2425_group_24_a2/detection.h>
// import for tiago eyes callback
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// import for ObjectDetectionCallback
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <cstdint>
// import for moveIt 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
// import for TF
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class NodeB
{
	public:

		explicit NodeB(ros::NodeHandle &nh) : nh_(nh), it(nh_), tf_listener(tf_buffer)
        {
			// Initialize the service server
			ROS_INFO("Node_B server is up and ready to receive requests.");
			// Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &NodeB::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeB::ObjectDetectionCallback, this);
            // publisher for the initial tilt of the camera 
            activated = false;
			picking_pub = nh_.advertise<ir2425_group_24_a2::picking_completed>("/picking_terminated", 10);
            activate_detection_sub = nh_.subscribe("/start_detection", 10, &NodeB::ActivateDetectionCallBack, this);
		}


		// Callback for message from NodeA
		void ActivateDetectionCallBack(const ir2425_group_24_a2::detection::ConstPtr& msg) {
			if(msg->activate_detection)
			{
				activated = true;
                ros::Duration(1.0).sleep();
                activated = false;
				return;
			}
			else if(msg->start_picking){
				CollisionTable();
				ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));
				bool success = planning_scene_interface.applyCollisionObjects(collision_objects);
				if(success) ROS_INFO("Successfully added collision objects");
				else ROS_WARN("Failed to add the collision objects");
				return;
			}
            return;
		}


	private:

        ros::NodeHandle &nh_;
         image_transport::ImageTransport it;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
		// Service server
		ros::ServiceServer service_;
		// vector for storing the detected objects frames
		ros::Subscriber object_detection_sub; // subscriber for apriltag detection
		image_transport::Subscriber image_sub; // subscriber for camera
		bool activated; // variable to trigger the detection of the objects
        // Initialize MoveIt Planning Scene Interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        int id;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
		ros::Publisher picking_pub;
		ros::Subscriber activate_detection_sub;

				
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
                //resize(img, resized_down, cv::Size(down_width, down_height), cv::INTER_LINEAR);
                        // Display the image
                        cv::imshow("Tiago Eyes", img);
                        cv::waitKey(1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
            }
        }

		// CallBack for Object Detection
        // ______________________________________________
        // Callback that continuously search for AprilTags
        // in Tiago's camera and compute their position
        // wrt map frame (20 Hz)
        void ObjectDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
            if(msg->detections.empty()){
                return;
            }

            // callBack always running, saving detected poses only when required
            if (activated) {
                id = msg->detections[0].id[0];
                for(const auto& object : collision_objects){
                    if(stoi(object.id)==id){
                        return;
                    }
                }
				geometry_msgs::PoseStamped frame;
				frame.header.seq = static_cast<uint32_t>(id); // saving ID
                frame.header.frame_id = msg->detections[0].pose.header.frame_id; // Get frame_id from the detection
				ROS_INFO("Detected object ID: %u",frame.header.seq);
				frame.pose.position = msg->detections[0].pose.pose.pose.position; // saving position
				frame.pose.orientation = msg->detections[0].pose.pose.pose.orientation;// saving orientation	
                // Add as collision object
                moveit_msgs::CollisionObject collision_object;
                collision_object.header.frame_id = "map";  // Use map frame
                collision_object.id = std::to_string(id);

                // from camera frame to map frame 
                // Define a transform from the camera frame (or detected frame) to the map frame
                geometry_msgs::PoseStamped frame_in_map;

                tf2_ros::TransformListener tf_listener(tf_buffer);

                ros::Rate rate(100.0);  // Loop frequency in Hz
                while (ros::ok()) {
                    ROS_INFO("Waiting transform from map to %s",frame.header.frame_id.c_str());
                    if(tf_buffer.canTransform("map", frame.header.frame_id, ros::Time::now(), ros::Duration(1.0))) {
                        try {
                            tf_buffer.transform(frame, frame_in_map, "map", ros::Duration(0.1));
                            ROS_INFO("Successfully transformed frame from %s to map frame", frame.header.frame_id.c_str());
                            break;  // Exit loop after successful transformation
                        } catch (tf2::TransformException &ex) {
                            ROS_WARN("Could not transform pose from %s to map frame: %s", frame.header.frame_id.c_str(), ex.what());
                        }
                    } else {
                        //ROS_WARN_THROTTLE(1.0, "Transform not available from %s to map frame. Retrying...", frame.header.frame_id.c_str());
                    }
                    //ros::Duration(0.1).sleep();  // Sleep for 0.1 seconds between checks
                    rate.sleep();
                    
                }
                frame_in_map.pose.orientation.x = 0;
                frame_in_map.pose.orientation.y = 0;


                // Define collision object shape (e.g., cylinder for hexagonal prism)
                shape_msgs::SolidPrimitive primitive;
                if (id <= 3) {  // Hexagonal prism
                    primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 0.22;  // Increased height
                    primitive.dimensions[1] = 0.055; // Increased radius
                    frame_in_map.pose.position.z = 0.90+0.11;  // Updated z-correction
                } else if (id <= 6) { // Cube
                    primitive.type = shape_msgs::SolidPrimitive::BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = 0.055; // Increased X size
                    primitive.dimensions[1] = 0.055; // Increased Y size
                    primitive.dimensions[2] = 0.055; // Increased Z size
                    frame_in_map.pose.position.z = 0.90+0.0275;  // Updated z-correction
                } else { // Triangular prism
                    primitive.type = shape_msgs::SolidPrimitive::BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = 0.077;  // Increased X size (Length)
                    primitive.dimensions[1] = 0.055;  // Increased Y size (Base)
                    primitive.dimensions[2] = 0.0385; // Increased Z size (Height)
                    frame_in_map.pose.position.z = 0.90+0.01925;  // Updated z-correction
                }


                // Assign pose
                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(frame_in_map.pose);
                collision_object.operation = moveit_msgs::CollisionObject::ADD;

                // Add to the planning scene
                collision_objects.push_back(collision_object);
            }
		}

        void CollisionTable() {
            std::vector<moveit_msgs::CollisionObject> collision_objects;

            // Table dimensions (slightly larger than real ones for safety)
            double table_size = 0.95; 

            // Pick-up table
            moveit_msgs::CollisionObject pickup_table;
            pickup_table.header.frame_id = "map";  // Use map as the reference frame
            pickup_table.id = "pickup_table";

            // Define cube dimensions
            shape_msgs::SolidPrimitive pickup_table_primitive;
            pickup_table_primitive.type = shape_msgs::SolidPrimitive::BOX;
            pickup_table_primitive.dimensions.resize(3);
            pickup_table_primitive.dimensions[0] = table_size; // Length
            pickup_table_primitive.dimensions[1] = table_size; // Width
            pickup_table_primitive.dimensions[2] = 0.90; // Height

            // Define the pose
            geometry_msgs::Pose pickup_table_pose;
            pickup_table_pose.position.x = 7.78904; // Adjust based on the workspace
            pickup_table_pose.position.y = -3.01049; // Adjust based on the workspace
            pickup_table_pose.position.z = 0.45; // Half the height of the table for the center point

            // Assign primitive and pose to the collision object
            pickup_table.primitives.push_back(pickup_table_primitive);
            pickup_table.primitive_poses.push_back(pickup_table_pose);
            pickup_table.operation = moveit_msgs::CollisionObject::ADD;

            collision_objects.push_back(pickup_table);

            ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));

            // Apply the collision objects to the planning scene
            bool success = planning_scene_interface.applyCollisionObjects(collision_objects);

            if(success){
                    ROS_INFO("Successfully added Table collision object");
                }
            else{
                ROS_WARN("Failed to add Table collision object");
            }
        }



};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "Node_B");
    ros::Rate rate(20);
    ros::NodeHandle nh;
    // Create an instance of the server class
    NodeB node_b_server(nh);

    while (ros::ok()) {
        ros::spinOnce(); // Process callbacks
        rate.sleep();
    }

    return 0;
}
