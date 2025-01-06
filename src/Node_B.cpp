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
#include <string>
#include <cstdint>
// import for moveIt 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
// import for TF
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// import fro camera tilt
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class NodeB
{
	public:

		NodeB() : head_client("/head_controller/follow_joint_trajectory", true)
        {
			// Initialize the service server
			service_ = nh_.advertiseService("picking", &NodeB::DetectionCallback, this);
			ROS_INFO("Node_B server is up and ready to receive requests.");
			// Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_); // image transport for the camera topic
            image_sub = it.subscribe("/xtion/rgb/image_color", 100, &NodeB::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeB::ObjectDetectionCallback, this);
            // publisher for the initial tilt of the camera 
            // to be ready to detect aprilTags
            tilt_cam_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
            activated = false;
            // Wait for the action server to be available
            ROS_INFO("Waiting for head action server to start...");
            head_client.waitForServer();
		}

	private:

		// Node handle
		ros::NodeHandle nh_;
		// Service server
		ros::ServiceServer service_;
		// vector for storing the detected objects frames
		ros::Subscriber object_detection_sub; // subscriber for apriltag detection
		image_transport::Subscriber image_sub; // subscriber for camera
		bool activated; // variable to trigger the detection of the objects
        // Initialize MoveIt Planning Scene Interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        int id;
        tf2_ros::Buffer tf_buffer;
        ros::Publisher tilt_cam_pub; // publisher for the initial tilt of the camera
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        TrajectoryClient head_client;


		// Callback for the service
		bool DetectionCallback(ir2425_group_24_a2::picking::Request &req, ir2425_group_24_a2::picking::Response &res) {
			
            ROS_INFO("Received request from Node_A!");
            // Clear the vector
            collision_objects.clear();
            CollisionTable();
			activated = true;
            ros::Duration(1.0).sleep();
            MoveCamera();

            activated = false;
            ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));
            bool success = planning_scene_interface.applyCollisionObjects(collision_objects);
            if(success){
                ROS_INFO("Successfully added all collision objects");
            }
            else{
                ROS_WARN("Failed to add the collision objects");
            }

			// Process the request and populate the response
			res.picked_obj_id = id; 
			ROS_INFO("Object ID = %d picking action completed", res.picked_obj_id);

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



                // Define collision object shape (e.g., cylinder for hexagonal prism)
                shape_msgs::SolidPrimitive primitive;
                if (id <= 3) {  // hexagonal prism
                    primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
                    primitive.dimensions.resize(2);
                    primitive.dimensions[0] = 0.25; // Height of the cylinder
                    primitive.dimensions[1] = 0.055; // Radius of the cylinder
                    frame_in_map.pose.position.z -= 0.125;
                } else if(id <= 6) { // cube
                    primitive.type = shape_msgs::SolidPrimitive::BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = 0.055; // X size
                    primitive.dimensions[1] = 0.055; // Y size
                    primitive.dimensions[2] = 0.055; // Z size
                    frame_in_map.pose.position.z -= 0.025;
                } else { // triangualr prism
                    primitive.type = shape_msgs::SolidPrimitive::BOX;
                    primitive.dimensions.resize(3);
                    primitive.dimensions[0] = 0.075; // X size
                    primitive.dimensions[1] = 0.055; // Y size
                    primitive.dimensions[2] = 0.0355; // Z size
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
            double table_size = 0.760574; 

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
            pickup_table_primitive.dimensions[2] = table_size; // Height

            // Define the pose
            geometry_msgs::Pose pickup_table_pose;
            pickup_table_pose.position.x = 7.83904; // Adjust based on the workspace
            pickup_table_pose.position.y = -3.11049; // Adjust based on the workspace
            pickup_table_pose.position.z = table_size/2; // Half the height of the table for the center point

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

        /*
        void MoveCamera(double pan, double tilt){
            ros::Rate loop_rate(10);
            ros::Time start_time = ros::Time::now(); // Record the start time

            while (ros::ok() && (ros::Time::now() - start_time).toSec() < 2.0) {
                // Create a JointTrajectory message
                trajectory_msgs::JointTrajectory joint_trajectory_msg;
                joint_trajectory_msg.joint_names.push_back("head_1_joint"); // Add the pan joint
                joint_trajectory_msg.joint_names.push_back("head_2_joint"); // Add the tilt joint

                // Create a trajectory point
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions.push_back(pan); // Pan position (e.g., 0 radians)
                point.positions.push_back(tilt); // Tilt position (e.g., 0.2 radians)
                point.velocities.push_back(0.0); // Pan velocity
                point.velocities.push_back(0.0); // Tilt velocity
                point.time_from_start = ros::Duration(1.0); // Duration for the motion

                // Add the point to the trajectory
                joint_trajectory_msg.points.push_back(point);

                // Publish the message
                tilt_cam_pub.publish(joint_trajectory_msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }*/
        void MoveCamera() {
            // Set initial pan and tilt values (e.g., 0 radians for pan, 45 degrees down for tilt)
            double initial_pan = 0.0; // Pan in radians (0 radians is center)
            double initial_tilt = -M_PI / 4.0; // Tilt in radians (-45 degrees)
            
            // Create a FollowJointTrajectoryGoal message (for control_msgs)
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.joint_names.push_back("head_1_joint"); // Pan joint
            goal.trajectory.joint_names.push_back("head_2_joint"); // Tilt joint

            // Define the sequence of trajectory points
            std::vector<trajectory_msgs::JointTrajectoryPoint> points;

            // Start position: 45 degrees down (initial tilt position)
            trajectory_msgs::JointTrajectoryPoint start_point;
            start_point.positions.push_back(initial_pan); // Keep the initial pan
            start_point.positions.push_back(initial_tilt); // 45 degrees down
            start_point.time_from_start = ros::Duration(2.0); // Move to this position in 2 seconds
            points.push_back(start_point);
             // Add all points to the trajectory message
            goal.trajectory.points = points;
            // Send the goal (trajectory message)
            head_client.sendGoal(goal);
            points.clear();
            head_client.waitForResult(ros::Duration(7.0));



            // Look slightly left
            trajectory_msgs::JointTrajectoryPoint left_point;
            left_point.positions.push_back(initial_pan - 0.3); // Pan slightly to the left
            left_point.positions.push_back(initial_tilt + 0.2); // Slightly raise the head
            left_point.time_from_start = ros::Duration(2.0); // 2 seconds later
            points.push_back(left_point);
             // Add all points to the trajectory message
            goal.trajectory.points = points;
            // Send the goal (trajectory message)
            head_client.sendGoal(goal);
            points.clear();
            head_client.waitForResult(ros::Duration(7.0));

            // Look slightly up
            trajectory_msgs::JointTrajectoryPoint up_point;
            up_point.positions.push_back(initial_pan); // Return to center pan
            up_point.positions.push_back(initial_tilt + 0.4); // Raise head further
            up_point.time_from_start = ros::Duration(2.0); // 2 seconds later
            points.push_back(up_point);
             // Add all points to the trajectory message
            goal.trajectory.points = points;
            // Send the goal (trajectory message)
            head_client.sendGoal(goal);
            points.clear();
            head_client.waitForResult(ros::Duration(7.0));

            // Look slightly right
            trajectory_msgs::JointTrajectoryPoint right_point;
            right_point.positions.push_back(initial_pan + 0.3); // Pan slightly to the right
            right_point.positions.push_back(initial_tilt + 0.2); // Lower head slightly
            right_point.time_from_start = ros::Duration(2.0); // 2 seconds later
            points.push_back(right_point);
             // Add all points to the trajectory message
            goal.trajectory.points = points;
            // Send the goal (trajectory message)
            head_client.sendGoal(goal);
            points.clear();
            head_client.waitForResult(ros::Duration(7.0));

            // Return to center position
            trajectory_msgs::JointTrajectoryPoint center_point;
            center_point.positions.push_back(initial_pan); // Center pan
            center_point.positions.push_back(initial_tilt); // Return to initial tilt
            center_point.time_from_start = ros::Duration(2.0); // 2 seconds later
            points.push_back(center_point);
             // Add all points to the trajectory message
            goal.trajectory.points = points;
            // Send the goal (trajectory message)
            head_client.sendGoal(goal);
            points.clear();
            head_client.waitForResult(ros::Duration(7.0));
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
