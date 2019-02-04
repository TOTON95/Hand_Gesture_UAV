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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>

#define PI 3.141592653589793238462

double heading;								//Drone's heading
int btn_emergency;							//Physical emergency's button
geometry_msgs::Twist cmd_vel_bebop;					//Contains the velocity information to be sent

double ce_hdg, ce_pos_X, ce_pos_Y, ce_alt;				//Control efforts delivered by the PD controller
std_msgs::Float64 st_pos_X,st_pos_Y,st_alt,st_hdg;    			//State variables
std_msgs::Float64 stp_pos_X,stp_pos_Y,stp_alt,stp_hdg;                  //Setpoint variables
std::vector<hand_gesture_uav::BoundingBox> bb;				//Vector with YOLO's Bounding Boxes
std::vector<geometry_msgs::Point> waypoints;				//Vector with waypoints
geometry_msgs::Point hand;						//Point that stores the location of the hand

cv::Mat depth;								//Depth frame
cv::Mat rgb;								//RGB frame
cv::Point p_hand;							//Point hand in the image

tf::StampedTransform transform_torso; 					//Tracking of the torso
tf::StampedTransform transform_left_hand;				//Tracking of the left hand
tf::StampedTransform transform_right_hand;				//Tracking of the right hand

struct v_object                                                         //Structure that describes the properties of the object
{
        double _posX,_posY,_posZ;                                       //Position of the drone
        double _errorX,_errorY,_errorZ;                                 //Error of the position of the drone
        double _orX,_orY,_orZ,_orW;                                     //Orientation of the drone
        double _roll,_pitch,_yaw,_yawRAD;                               //Roll,Pitch,Yaw (degrees), Yaw (radians)
        double _cmdX,_cmdY,_cmdZ,_cmdYAW;                               //Command values
        double rot_cmd_x,rot_cmd_y;                                     //Position in the rotated matrix
        double _velX,_velY,_velZ,_velYAW;                               //Velocities
        double abs_x,abs_y;                                             //Absolute position in X and Y
        double angle_res,angle_arc;                                     //Angle resultant, angle of the arc
}bebop;

void getJoyState(const sensor_msgs::Joy::ConstPtr& js)                  //Function to obtain the data from the emergency button of the joystick
{
        btn_emergency = js->buttons[0];
}

void getBebopPos(const geometry_msgs::TransformStamped::ConstPtr& pos)  //Function to obtain the position from the vicon system
{
        bebop._posX = pos->transform.translation.x;                     //Position in X
        bebop._posY = pos->transform.translation.y;                     //Position in Y
        bebop._posZ = pos->transform.translation.z;                     //Position in Z
        bebop._orX = pos->transform.rotation.x;                         //Rotation in X
        bebop._orY = pos->transform.rotation.y;                         //Rotation in Y
        bebop._orZ = pos->transform.rotation.z;                         //Rotation in Z
        bebop._orW = pos->transform.rotation.w;                         //Rotation in W

        tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(bebop._roll,bebop._pitch,bebop._yawRAD);               //Get the Roll, Pitch, Yaw (Radians)
        bebop._yaw = bebop._yawRAD*(180/PI);                            //Convert the Yaw (Radians) into Yaw (Degrees)
        heading = bebop._yaw;                                           //Set the heading of the drone  


        bebop.abs_x = bebop._posX;                                      //Set the absolute position of the drone in X
        bebop.abs_y = bebop._posY;                                      //Set the absolute position of the drone in Y
}

//Avoids problems with the heading of the motion capture system

double GetAngleDifference(double from, double to)
{
        double difference = to - from;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
}

void printTF(tf::StampedTransform& transform)
{
    ROS_INFO("%s to %s: x: %lf, y: %lf, z: %lf;  x: %lf y: %lf z: %lf w: %lf", transform.frame_id_.c_str(), \
            transform.child_frame_id_.c_str(), transform.getOrigin().x(), \
            transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), \
            transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
}

//Get Control Efforts

void getEffort_pos_X(const std_msgs::Float64::ConstPtr& msg)
{
        ce_pos_X = msg->data;
}
void getEffort_pos_Y(const std_msgs::Float64::ConstPtr& msg)
{
        ce_pos_Y = msg->data;
}
void getEffort_alt(const std_msgs::Float64::ConstPtr& msg)
{
        ce_alt = msg->data;
}
void getEffort_hdg(const std_msgs::Float64::ConstPtr& msg)
{
        ce_hdg = -msg->data;
}

void getDepthCameraInfo()
{
	if(!depth.empty())
	{	
		cv::imshow("Depth",depth);
		cv::waitKey(27);
	}
}

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
			cv::putText(rgb, str ,cv::Point(20, 20),cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,255,0)); 
	}		
}

void getRGBCameraInfo()
{
	if(!rgb.empty())
        {
		getCurrentPoint();
                cv::imshow("rgb",rgb);
                cv::waitKey(27);
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

//Yolo callback
void cb_yolo(const hand_gesture_uav::BoundingBoxes::ConstPtr& msg)
{
	bb = msg -> bounding_boxes;
	for(int i=0; i < bb.size(); i++)
	{
		if(bb[i].Class=="A")
		{
			//TODO: Play Sequence
		}
		if(bb[i].Class=="B")
                {
                        //TODO: Set Waypoint
                }
		if(bb[i].Class=="C")
                {
                        //TODO: Delete Waypoints
                }
		if(bb[i].Class=="D")
                {
                        //TODO: Stop Sequence
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
	ros::Subscriber joy_sub,bebop_sub;				//Joystick and Vicon subs
	ros::Subscriber pid_pos_X,pid_pos_Y,pid_hdg,pid_alt;		//PID controllers subs
	ros::Subscriber yolo_sub;					//Yolo node sub
	image_transport::Subscriber kinect_sub;				//Kinect image subscriber
	image_transport::Subscriber rgb_sub;				//Kinect rgb subscriber
	ros::Publisher state_pos_X,state_pos_Y,state_hdg,state_alt;	//Current state of the drone
	ros::Publisher sp_pos_X,sp_pos_Y,sp_hdg,sp_alt;			//Set the goal of the drone
	ros::Publisher tko,land;					//Take-off and landing publisher
	ros::Publisher cmd_vel;						//Velocity command of the drone

	tf::TransformListener listener_torso;				//Listener of tf states for the torso
	tf::TransformListener listener_l_hand;				//Listener of tf states for the left hand
	tf::TransformListener listener_r_hand;				//Listener of tf states for the right hand
	
	//Subscribers of the ROS node
	joy_sub = n.subscribe("/joy",1000,getJoyState);
	bebop_sub = n.subscribe("/vicon/BEBOP_1_11_2_18/BEBOP_1_11_2_18",1000,getBebopPos);
	pid_pos_X = n.subscribe("/bebop_hand/control_effort_pos_X",1000,getEffort_pos_X);
        pid_pos_Y = n.subscribe("/bebop_hand/control_effort_pos_Y",1000,getEffort_pos_Y);
        pid_alt = n.subscribe("/bebop_hand/control_effort_alt",1000,getEffort_alt);
        pid_hdg = n.subscribe("/bebop_hand/control_effort_hdg",1000,getEffort_hdg);
        cmd_vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
	yolo_sub = n.subscribe("/darknet_ros/bounding_boxes",30,cb_yolo);
	kinect_sub = it_.subscribe("/camera/depth_registered/sw_registered/image_rect",1,getKinectImage);
	rgb_sub = it_.subscribe("/camera/rgb/image_rect_color",1,getRGBImage);

	//Publishers of the ROS node
	tko = n.advertise<std_msgs::Empty>("/bebop/takeoff",1000);
	land = n.advertise<std_msgs::Empty>("/bebop/land",1000);
	state_pos_X = n.advertise<std_msgs::Float64>("/bebop_hand/state_pos_X",1000);
        state_pos_Y = n.advertise<std_msgs::Float64>("/bebop_hand/state_pos_Y",1000);
        state_alt = n.advertise<std_msgs::Float64>("/bebop_hand/state_alt",1000);
        state_hdg = n.advertise<std_msgs::Float64>("/bebop_hand/state_hdg",1000);
        sp_pos_X = n.advertise<std_msgs::Float64>("/bebop_hand/setpoint_pos_X",1000);
        sp_pos_Y = n.advertise<std_msgs::Float64>("/bebop_hand/setpoint_pos_Y",1000);
        sp_alt = n.advertise<std_msgs::Float64>("/bebop_hand/setpoint_alt",1000);
        sp_hdg = n.advertise<std_msgs::Float64>("/bebop_hand/setpoint_hdg",1000);
	
	std_msgs::Empty msg_tko,msg_land;				 //Msgs to take-off and land

	//Making sure to set every velocity to 0
       	cmd_vel_bebop.linear.x = 0;
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;
	cmd_vel_bebop.angular.x = 0;
	cmd_vel_bebop.angular.y = 0;
	cmd_vel_bebop.angular.z = 0;

	ros::spinOnce();						 //Refresh the topics
	double hdg_target = 0;						 //Heading target

	ros::Duration(2).sleep();					 //Time necessary to setup
	ros::Rate r(100);						 //Set node at 100Hz

	printf("\n========== T A K E O F F ==========\n");
	tko.publish(msg_tko);						 //Take-off
	ros::Duration(4.5).sleep();					 //Wait until the drone elevates
		
	//Stops the node once that <Ctrl + C > is pressed
	while(n.ok())
	{
		//Lands the drone if the joystick's button is pressed
		if(btn_emergency)
		{
			ros::Duration(0.525).sleep();
			printf("\n========== L A N D [ J S ]==========\n");
                        land.publish(msg_land);                           //Land the drone
                        break;
		}

		getYoloInfo();						  //Yolo objdet info
		getDepthCameraInfo();					  //Depth image retrieval
		getRGBCameraInfo();					  //RGB image retrieval

		try
		{
			listener_torso.lookupTransform("/openni_depth_frame","torso_1",ros::Time(0),transform_torso);
			listener_l_hand.lookupTransform("/openni_depth_frame","left_hand_1",ros::Time(0),transform_left_hand);
			listener_r_hand.lookupTransform("/openni_depth_frame","right_hand_1",ros::Time(0),transform_right_hand);
			
			//printTF(transform_torso);
			//printTF(transform_left_hand);
			//printTF(transform_right_hand);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}


		//Wrapping the angle 
		double diff = GetAngleDifference(heading,hdg_target);

		//Updating Heading PID Controller
		st_hdg.data=diff;
                stp_hdg.data=0;
                ros::spinOnce();

                state_hdg.publish(st_hdg);
                sp_hdg.publish(stp_hdg);
                ros::Duration(0.0001).sleep();

                ros::spinOnce();

		//Set heading velocity
                cmd_vel_bebop.angular.z = ce_hdg;

		//Updating Position PID Controller
		ros::spinOnce();

                st_pos_X.data=bebop._posX;
                stp_pos_X.data=0;
                state_pos_X.publish(st_pos_X);
                sp_pos_X.publish(stp_pos_X);
                ros::spinOnce();
                bebop._cmdX = ce_pos_X;	
		
		st_pos_Y.data=bebop._posY;
                stp_pos_Y.data=0;
                state_pos_Y.publish(st_pos_Y);
                sp_pos_Y.publish(stp_pos_Y);
                ros::spinOnce();
                bebop._cmdY = ce_pos_Y;

		//Calculating the rotation matrix of the drone
		bebop.rot_cmd_x = bebop._cmdX*cos(constrainAngle(heading) * 0.0174533) + bebop._cmdY*sin(constrainAngle(heading) * 0.0174533);
                bebop.rot_cmd_y = bebop._cmdX*sin(constrainAngle(heading) * 0.0174533) - bebop._cmdY*cos(constrainAngle(heading) * 0.0174533);

		//Set X and Y velocities
		cmd_vel_bebop.linear.x = bebop.rot_cmd_x;
		cmd_vel_bebop.linear.y = -bebop.rot_cmd_y;

		//Updating Altittude PID Controller
		st_alt.data = bebop._posZ;
		stp_alt.data = 1.05;
		state_alt.publish(st_alt);
		sp_alt.publish(stp_alt);
		ros::spinOnce();

		//Set velocity in Z
		cmd_vel_bebop.linear.z = ce_alt;
		
		//Send the velocity command
		cmd_vel.publish(cmd_vel_bebop);

		//ROS_INFO("X: %lf  Y: %lf HDG: %lf CMDX: %lf  CMDY: %lf CMDHDG: %lf",bebop._posX,bebop._posY, heading, cmd_vel_bebop.linear.x,cmd_vel_bebop.linear.y,cmd_vel_bebop.angular.z);

		ros::spinOnce();

		r.sleep();

	}

	ros::Duration(2).sleep();					//Wait 2 Seconds to finish
	ros::spinOnce();
	
	//Land the drone if it still flying
	while(bebop._posZ > 0.60)
	{
		ros::Duration(0.525).sleep();
		printf("\n========== L A N D [ A L T ]==========\n");
		land.publish(msg_land);
	}

	return 0;
}

