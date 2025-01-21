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
// import for action client node_B
#include <actionlib/client/simple_action_client.h> 
#include <ir2425_group_24_a2/manipulationAction.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> // For TF message




typedef actionlib::SimpleActionClient<ir2425_group_24_a2::manipulationAction> PickingClient;

class NodeB
{
	public:

		NodeB():picking_client_("/picking", true) // action client to the move_base action server
        {
			// Initialize the service server
			ROS_INFO("Node_B server is up and ready to receive requests.");
			// Subscriber of image_transport type for Tiago Camera-Visual topic (essages rate: 30 Hz)
            image_transport::ImageTransport it(nh_);
            image_sub = it.subscribe("/xtion/rgb/image_color", 10, &NodeB::tiagoEyesCallback, this);
            // Subscriber to the AprilTag detection topic (messages rate: 20 Hz)
            object_detection_sub = nh_.subscribe("/tag_detections", 10, &NodeB::ObjectDetectionCallback, this);
            // Wait for the NodeC picking action server to start
            ROS_INFO("Waiting for the /picking action server to start...");
            picking_client_.waitForServer();
            ROS_INFO("/picking action server started.");
            activated = false;
            // publisher to send the termination of the whole picking action
			picking_pub = nh_.advertise<ir2425_group_24_a2::picking_completed>("/picking_terminated", 10);
            // subscriber to the callback to receive when to detect objects
            activate_detection_sub = nh_.subscribe("/start_detection", 10, &NodeB::ActivateDetectionCallBack, this);
            table_h = 0.78;
		}


		// Callback for communication with Node_A
		void ActivateDetectionCallBack(const ir2425_group_24_a2::detection::ConstPtr& msg) {
            // msg to stop the detection and initialize the picking operation
            if(msg->start_picking){
                CollisionTable(); // add the table as collision object
                // apply all the collision objects
				ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", ros::Duration(5.0));
				bool success = planning_scene_interface.applyCollisionObjects(collision_objects);
				if(success) ROS_INFO("Successfully added all collision objects");
				else ROS_WARN("Failed to add the collision objects");
                initialize_picking();
				return;
            }
            // msg to start detection
			if(msg->activate_detection)
			{
				activated = true;
				return;
			}
            // msg to stop detection 
            else{
                activated = false;
                return;
            }
		}


	private:

        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer; 
		ros::Subscriber object_detection_sub; // subscriber for apriltag detection
		image_transport::Subscriber image_sub; // subscriber for camera display
		bool activated; // variable to trigger the detection of the objects or not
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // planning scene for the collision objects  
        std::vector<moveit_msgs::CollisionObject> collision_objects; // vector containing all collision objects detected
		ros::Publisher picking_pub; // publisher for sending the termination msg
		ros::Subscriber activate_detection_sub; // subscriber to receive the start msg
        PickingClient picking_client_; // action client for the picking action
        double table_h;
        // send the goal of the collision object
        ir2425_group_24_a2::manipulationGoal goal;
        tf2_ros::TransformBroadcaster tf_broadcaster_;


				
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
                int id;
                for(const auto& detection : msg->detections){
                    id = detection.id[0];
                    // verify that it wasn't already detected
                    for(const auto& object : collision_objects){
                        if(stoi(object.id)==id){
                            return;
                        }
                    }
                    // get the pose of the object
                    geometry_msgs::PoseStamped frame;
                    frame.header.seq = static_cast<uint32_t>(id); // saving ID
                    frame.header.frame_id = detection.pose.header.frame_id; // Get frame_id from the detection
                    ROS_INFO("Detected object ID: %u",frame.header.seq);
                    frame.pose.position = detection.pose.pose.pose.position; // saving position
                    frame.pose.orientation = detection.pose.pose.pose.orientation;// saving orientation	
                    // Define a transform from the camera frame (or detected frame) to the map frame
                    geometry_msgs::PoseStamped frame_in_bf;
                    tf2_ros::TransformListener tf_listener(tf_buffer);
                    ros::Rate rate(100.0);  // Loop frequency in Hz
                    // transform from camera frame to map frame
                    while (ros::ok()) {
                        if(tf_buffer.canTransform("base_footprint", frame.header.frame_id, ros::Time::now(), ros::Duration(1.0))) {
                            try {
                                tf_buffer.transform(frame, frame_in_bf, "base_footprint", ros::Duration(0.1));
                                break;  // Exit loop after successful transformation
                            } catch (tf2::TransformException &ex) {
                                ROS_WARN("Could not transform pose from %s to base_footprint frame: %s", frame.header.frame_id.c_str(), ex.what());
                            }
                        } 
                        rate.sleep();  
                    }
                    // Add the object as collision object
                    moveit_msgs::CollisionObject collision_object;
                    collision_object.header.frame_id = "base_footprint";  // Use map frame
                    collision_object.id = std::to_string(id); 
                    // fixed orientations on x and y because we 
                    // assume that the objects are placed over 
                    // the table (same reason for the z-correction
                    // in position)
                    //frame_in_bf.pose.orientation.x = 0;
                    //frame_in_bf.pose.orientation.y = 0;
                    // Define collision object shape 
                    shape_msgs::SolidPrimitive primitive;
                    if (id <= 3) {  // Hexagonal prism
                        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
                        primitive.dimensions.resize(2);
                        primitive.dimensions[0] = 0.22;  // height
                        primitive.dimensions[1] = 0.035; // radius
                        frame_in_bf.pose.position.z = table_h + 0.11; // z-correction
                    } else if (id <= 6) { // Cube
                        primitive.type = shape_msgs::SolidPrimitive::BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = 0.055; // (Length)
                        primitive.dimensions[1] = 0.055; // (Base)
                        primitive.dimensions[2] = 0.055; // (Height)
                        frame_in_bf.pose.position.z = table_h + 0.0275; // z-correction
                    } else { // Triangular prism
                        primitive.type = shape_msgs::SolidPrimitive::BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = 0.055;  // (Length)
                        primitive.dimensions[1] = 0.077;  // (Base)
                        primitive.dimensions[2] = 0.0385; // (Height)
                        frame_in_bf.pose.position.z = table_h + 0.01925;  // z-correction
                    }

                    // add to the collision objects
                    collision_object.primitives.push_back(primitive);
                    collision_object.primitive_poses.push_back(frame_in_bf.pose);
                    collision_object.operation = moveit_msgs::CollisionObject::ADD;
                    collision_objects.push_back(collision_object);
                }
            }
		}

        // method to add the table as a collision object
        void CollisionTable() {

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
            pickup_table_primitive.dimensions[2] = table_h; // Height

            // Define the pose
            geometry_msgs::Pose pickup_table_pose;
            //pickup_table_pose.position.x = 7.88904; // Adjust based on the workspace
            //pickup_table_pose.position.y = -2.99049; // Adjust based on the workspace
            pickup_table_pose.position.x = 0.82; // Adjust based on the workspace
            pickup_table_pose.position.y = 0.02; // Adjust based on the workspace
            pickup_table_pose.position.z = 0.375; // Half the height of the table for the center point

            // Assign primitive and pose to the collision object
            pickup_table.primitives.push_back(pickup_table_primitive);
            pickup_table.primitive_poses.push_back(pickup_table_pose);
            pickup_table.operation = moveit_msgs::CollisionObject::ADD;

            collision_objects.push_back(pickup_table);

        }

        // method to initialize the picking operation
        // choose the object to pick and send the goal
        // to Node_C to pick it!
        void initialize_picking()
        {
            // choose pickable object among the objects detected
            int id;
            for(const auto& object : collision_objects)
            {
                id = stoi(object.id);
                if(id >= 4 && id < 7){
                    // send the goal of the collision object
                    goal.ID = id;
                    goal.pose = object.primitive_poses[0];
                    // adjust z-axis to be the top of the object
                    if(id<=3){
                        goal.pose.position.z += 0.11;
                    }
                    else if(id<=6){
                        goal.pose.position.z += 0.0275;
                    }
                    else{
                        goal.pose.position.z += 0.01925;
                    } 
                    picking_client_.sendGoal(goal);

                    // Monitor the result while allowing callbacks to process
                    while (!picking_client_.waitForResult(ros::Duration(0.1))) {
                        ros::spinOnce();  // Process other callbacks (e.g., Object Detection)
                    }
                    // Check if the goal was successfully executed
                    if (picking_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
                        ROS_WARN("Picking action failed");
                    else ROS_INFO("Picking action completed");
                    break;
                }
            }
            
            std::vector<std::string> names = planning_scene_interface.getKnownObjectNames();
            planning_scene_interface.removeCollisionObjects(names);
            // clear scene to not move it with the robot
            collision_objects.clear();
            
            // Check if the planning scene is now clear
            if (planning_scene_interface.getKnownObjectNames().empty()) {
                ROS_INFO("Successfully cleared the planning scene.");
            } else {
                ROS_WARN("Some collision objects could not be removed.");
            }
            // publish the message to NodeA to declare picking termination
            ir2425_group_24_a2::picking_completed msg; 
            msg.picking_completed = true;
            msg.id = id;
            picking_pub.publish(msg);
            ros::spinOnce();  
        }

};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "Node_B");
    // Create an instance of the server class
    NodeB node_b_server;
    ros::spin();

    return 0;
}
