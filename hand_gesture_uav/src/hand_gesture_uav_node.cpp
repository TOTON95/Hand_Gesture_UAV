#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <hand_gesture_uav/BoundingBoxes.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <hand_gesture_uav/BoundingBox.h>
#include <hand_gesture_uav/ArcDrone.h>
#include <hand_gesture_uav/Waypoint.h>
#include <hand_gesture_uav/Waypoints.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

std::vector<hand_gesture_uav::BoundingBox> bb;				//Vector with YOLO's Bounding Boxes
std::vector<geometry_msgs::Point> waypoints;				//Vector with waypoints
geometry_msgs::Point hand;						//Point that stores the location of the hand

cv::Mat depth;								//Depth frame
cv::Mat rgb;								//RGB frame
cv::Point p_hand;							//Point hand in the image

tf::StampedTransform transform_torso; 					//Tracking of the torso
tf::StampedTransform transform_left_hand;				//Tracking of the left hand
tf::StampedTransform transform_right_hand;				//Tracking of the right hand
tf::StampedTransform transform_head;					//Tracking of head

hand_gesture_uav::Waypoint wp;						//Waypoint to be sent to the uav

sensor_msgs::ImagePtr img_bridge;					//Republished image

image_transport::Publisher pub_cv_img;					//Image Publisher

geometry_msgs::Twist vel_drone, vel_hand;				//Drone and Hand velocities

unsigned int img_counter = 0; 

void printTF(tf::StampedTransform& transform)
{
    ROS_INFO("%s to %s: x: %lf, y: %lf, z: %lf;  x: %lf y: %lf z: %lf w: %lf", transform.frame_id_.c_str(), \
            transform.child_frame_id_.c_str(), transform.getOrigin().x(), \
            transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), \
            transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
}

void print_hand_torso_dist(tf::StampedTransform& hand, tf::StampedTransform& torso, tf::StampedTransform& head)
{
	double hand_y = hand.getOrigin().y();
	double torso_y = torso.getOrigin().y();
	double dist_y = hand_y - torso_y;
	
	double head_z = head.getOrigin().z();

	double hand_z = hand.getOrigin().z();
	double torso_z = torso.getOrigin().z();
	double dist_z = hand_z - torso_z;

	double dist_head_torso = head_z - torso_z;

	ROS_INFO("Distance from hand to torso:  y = %lf, z = %lf\n", dist_y, dist_z - 0.30);
	ROS_INFO("Distance from head to torso:  z = %lf\n", dist_head_torso);
}

void getDepthCameraInfo()
{
	if(!depth.empty())
	{	
		cv::imshow("Depth",depth);
		cv::waitKey(27);
	}
}

/*void drawWps()
{
	for(int i=0; i < points.size(); i++)
	{
		std::cout<<"B"<<std::endl;

		double radius = ((1 + 0.60)/ 1.70)*waypoints[i].x+1;
		cv::circle(rgb, points[i], radius*10, cv::Scalar(255,255,0),-1);
	}
}*/

void getCurrentPoint()
{
	if(!rgb.empty())
	{
			hand.x = transform_left_hand.getOrigin().x() - 1;
			hand.y = transform_left_hand.getOrigin().y();
			hand.z = transform_left_hand.getOrigin().z();

			p_hand.x = (int) ((rgb.cols/2) - ((rgb.cols/2)/0.85)*hand.y);
			p_hand.y = (int) ((rgb.rows/2) - ((rgb.rows/2)/0.85)*hand.z);
			double radius = ((1 + 0.60)/ 1.70)*hand.x+1;

			if(hand.x > 1.70) hand.x = 1.70;
			if(hand.x < -0.50) hand.x = -0.50;
			if(hand.y > 1.70) hand.y = 1.70;
			if(hand.y < -1.70) hand.y = -1.70;
			if(hand.z < 0.50) hand.z = 0.50;
			if(hand.z > 1.70) hand.z = 1.70;


		//	ROS_INFO("x: %d, y: %d, z: %lf",p_hand.x,p_hand.y,radius);
			cv::circle(rgb, p_hand, radius*10, cv::Scalar(0,255,0),-1);
		        std::string str = "X: " + std::to_string(hand.x)	+ " Y: " + std::to_string(hand.y) + " Z: " + std::to_string(hand.z);
			cv::putText(rgb, str ,cv::Point(20, 20),cv::FONT_HERSHEY_SIMPLEX, 0.95, cv::Scalar(0,255,0),3); 
	}		
}

void getRGBCameraInfo()
{
	if(!rgb.empty())
        {
		img_counter++;
		getCurrentPoint();

		std_msgs::Header header;
		header.seq = img_counter;
		header.stamp = ros::Time::now();
		cv_bridge::CvImage _img;
		_img.encoding = "bgr8";
		_img.image = rgb.clone(); 
		img_bridge = _img.toImageMsg();
		pub_cv_img.publish(img_bridge);
        }
}

void getRGBImage(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                rgb = cv_ptr->image.clone();
	}
	catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }

}

void getKinectImage(const sensor_msgs::ImageConstPtr& msg)
{
	try
        {
                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                depth = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }
}


void updateWaypoint()
{
	wp.wp.x = hand.x;
	wp.wp.y = hand.y;
	wp.wp.z = hand.z;
}

//Yolo callback
void cb_yolo(const hand_gesture_uav::BoundingBoxes::ConstPtr& msg)
{
	bb = msg -> bounding_boxes;
	for(int i=0; i < bb.size(); i++)
	{
		if(bb[i].Class=="A")
		{
			//TODO: Play Sequence
			//std::cout<<"A"<<std::endl;
			
		}
		if(bb[i].Class=="B")
                {

		}
		if(bb[i].Class=="C")
                {
			//std::cout<<"D"<<std::endl;
                        //TODO: Stop Sequence
			updateWaypoint();
                }
		if(bb[i].Class=="D")
                {
			//std::cout<<"D"<<std::endl;
                        //TODO: Stop Sequence
			updateWaypoint();
                }
	}
}

void getYoloInfo()
{
	//TODO: Add Yolo processing code
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_hand_gestures");			//Initiates the ROS node
	ros::NodeHandle n;						//Creates the node handle
	image_transport::ImageTransport it_ = image_transport::ImageTransport(n); //Creates the transport of images
	ros::Subscriber yolo_sub;					//Yolo node sub
	image_transport::Subscriber kinect_sub;				//Kinect image subscriber
	image_transport::Subscriber rgb_sub;				//Kinect rgb subscriber
	ros::Publisher wp_pub;						//Waypoint publisher

	tf::TransformListener listener_torso;				//Listener of tf states for the torso
	tf::TransformListener listener_l_hand;				//Listener of tf states for the left hand
	tf::TransformListener listener_r_hand;				//Listener of tf states for the right hand
	tf::TransformListener listener_head;				//Listener of tf states for the head
	
	//Subscribers of the ROS node
	yolo_sub = n.subscribe("/darknet_ros/bounding_boxes",30,cb_yolo);
	kinect_sub = it_.subscribe("/camera/depth_registered/sw_registered/image_rect",1,getKinectImage);
	rgb_sub = it_.subscribe("/camera/rgb/image_rect_color",1,getRGBImage);

	//Publisher ROS node
	wp_pub = n.advertise<hand_gesture_uav::Waypoint>("/bebop_wps/wp",1000);
	pub_cv_img = it_.advertise("/hgu/gui",100);

	ros::spinOnce();						 //Refresh the topics

	ros::Rate r(100);						 //Rate of the node

	printf("\n========== T A K E O F F ==========\n");
	ros::Duration(4.5).sleep();					 //Wait until the drone elevates
		
	int user = 1;

	//Stops the node once that <Ctrl + C > is pressed
	while(n.ok())
	{
		getYoloInfo();						  //Yolo objdet info
	//	getDepthCameraInfo();					  //Depth image retrieval
		getRGBCameraInfo();					  //RGB image retrieval

		try
		{
			std::string torso = "torso_" + std::to_string(user);
			std::string l_hand = "left_hand_" + std::to_string(user);
			std::string r_hand = "right_hand_" + std::to_string(user);
			std::string head = "head_" + std::to_string(user); 

			listener_torso.lookupTransform("/openni_depth_frame",torso,ros::Time(0),transform_torso);
			listener_l_hand.lookupTransform("/openni_depth_frame",l_hand,ros::Time(0),transform_left_hand);
			listener_r_hand.lookupTransform("/openni_depth_frame",r_hand,ros::Time(0),transform_right_hand);
			listener_head.lookupTransform("/openni_depth_frame",head,ros::Time(0),transform_head);	

			wp_pub.publish(wp);
			
			//printTF(transform_torso);
			//printTF(transform_left_hand);
			//printTF(transform_right_hand);

			print_hand_torso_dist(transform_left_hand,transform_torso,transform_head);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("Changing to other user, currently: %d" , user);
			ROS_ERROR("%s",ex.what());
			//print_hand_torso_dist(transform_left_hand,transform_torso);
			//user++;



			if(user>3) user=1;
		}

		ros::spinOnce();

		r.sleep();

	}
	return 0;
}

