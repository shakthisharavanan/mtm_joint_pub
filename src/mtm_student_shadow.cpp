#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class mtm_student
{
public:

	int switch_value; 
	mtm_student()
	{
		switch_value = 0; 
		teacher_joints_sub = n.subscribe<sensor_msgs::JointState>("/mtm_teacher/joint/states", 100, &mtm_student::joints_cb, this);
		teacher_switch_sub = n.subscribe<std_msgs::Int32>("/mtm_teacher_switch", 100, &mtm_student::switch_cb, this);

		outer_yaw_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_outer_yaw_joint/SetPositionTarget", 100);
		shoulder_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_shoulder_pitch_joint/SetPositionTarget", 100);
		elbow_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_elbow_pitch_joint/SetPositionTarget", 100);
		platform_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_platform_joint/SetPositionTarget", 100);
		pitch_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_pitch_joint/SetPositionTarget", 100);
		yaw_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_yaw_joint/SetPositionTarget", 100);
		roll_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_roll_joint/SetPositionTarget", 100);
	}

	void spin()
	{	
		ros::Rate loop_rate(100);
		cv::Mat img(300, 512, CV_8UC3);
		
		cv::namedWindow("Joints", WINDOW_AUTOSIZE);
		
		cv::createTrackbar("Outer Yaw (75)", "Joints", &slider, 120, nothing);
		cv::createTrackbar("Shoulder Pitch (45)", "Joints", &slider, 90, nothing);
		cv::createTrackbar("Elbow Pitch (45)", "Joints", &slider, 90, nothing);
		cv::createTrackbar("Wrist platform (60)", "Joints", &slider, 304, nothing);
		cv::createTrackbar("Wrist Pitch (90)", "Joints", &slider, 270, nothing);
		cv::createTrackbar("Wrist Yaw (45)", "Joints", &slider, 90, nothing);
		cv::createTrackbar("Wrist Roll (270)", "Joints", &slider, 540, nothing);

		cv::setTrackbarPos("Outer Yaw (75)", "Joints", 75);
		cv::setTrackbarPos("Shoulder Pitch (45)", "Joints", 45);
		cv::setTrackbarPos("Elbow Pitch (45)", "Joints", 45);
		cv::setTrackbarPos("Wrist platform (60)", "Joints", 60);
		cv::setTrackbarPos("Wrist Pitch (90)", "Joints", 90);
		cv::setTrackbarPos("Wrist Yaw (45)", "Joints", 45);
		cv::setTrackbarPos("Wrist Roll (270)", "Joints", 270);

		while (ros::ok())
		{
			ros::spinOnce();
			if (switch_value == 0)
			{	
				img = cv::Scalar(0,255,0);
				cv::imshow( "Joints", img); 
				cv::waitKey(1);

				outer_yaw.data = cv::getTrackbarPos("Outer Yaw (75)", "Joints");
				shoulder_pitch.data = cv::getTrackbarPos("Shoulder Pitch (45)", "Joints");
				elbow_pitch.data = cv::getTrackbarPos("Elbow Pitch (45)", "Joints");
				wrist_platform.data = cv::getTrackbarPos("Wrist platform (60)", "Joints");
				wrist_pitch.data = cv::getTrackbarPos("Wrist Pitch (90)", "Joints");
				wrist_yaw.data = cv::getTrackbarPos("Wrist Yaw (45)", "Joints");
				wrist_roll.data = cv::getTrackbarPos("Wrist Roll (270)", "Joints");

				outer_yaw.data = deg2rad(outer_yaw.data-75);
				shoulder_pitch.data = deg2rad(shoulder_pitch.data-45);
				elbow_pitch.data = deg2rad(elbow_pitch.data-45);
				wrist_platform.data = deg2rad(wrist_platform.data-60);
				wrist_pitch.data = deg2rad(wrist_pitch.data-90);
				wrist_yaw.data = deg2rad(wrist_yaw.data-45);
				wrist_roll.data = deg2rad(wrist_roll.data-270);

				outer_yaw_pub.publish(outer_yaw);
				shoulder_pub.publish(shoulder_pitch);
				elbow_pub.publish(elbow_pitch);
				platform_pub.publish(wrist_platform);
				pitch_pub.publish(wrist_pitch);
				yaw_pub.publish(wrist_yaw);
				roll_pub.publish(wrist_roll);

			}

			else
			{
				img = cv::Scalar(0,0,255);
				cv::imshow( "Joints", img); 
				cv:: waitKey(1);

				cv::setTrackbarPos("Outer Yaw (75)", "Joints", rad2deg(outer_yaw.data) + 75);
				cv::setTrackbarPos("Shoulder Pitch (45)", "Joints", rad2deg(shoulder_pitch.data) + 45);
				cv::setTrackbarPos("Elbow Pitch (45)", "Joints", rad2deg(elbow_pitch.data) + 45);
				cv::setTrackbarPos("Wrist platform (60)", "Joints", rad2deg(wrist_platform.data) + 60);
				cv::setTrackbarPos("Wrist Pitch (90)", "Joints", rad2deg(wrist_pitch.data) + 90);
				cv::setTrackbarPos("Wrist Yaw (45)", "Joints", rad2deg(wrist_yaw.data) + 45);
				cv::setTrackbarPos("Wrist Roll (270)", "Joints", rad2deg(wrist_roll.data) + 270);

				outer_yaw_pub.publish(outer_yaw);
				shoulder_pub.publish(shoulder_pitch);
				elbow_pub.publish(elbow_pitch);
				platform_pub.publish(wrist_platform);
				pitch_pub.publish(wrist_pitch);
				yaw_pub.publish(wrist_yaw);
				roll_pub.publish(wrist_roll);		
			}
		
		loop_rate.sleep();
		}
	}
	
private:

	ros::NodeHandle n;

	int slider;

	std_msgs::Float64 outer_yaw;
	std_msgs::Float64 shoulder_pitch;
	std_msgs::Float64 elbow_pitch;
	std_msgs::Float64 wrist_platform;
	std_msgs::Float64 wrist_pitch;
	std_msgs::Float64 wrist_yaw;
	std_msgs::Float64 wrist_roll;

	ros::Publisher outer_yaw_pub;
	ros::Publisher shoulder_pub;
	ros::Publisher elbow_pub;
	ros::Publisher platform_pub;
	ros::Publisher pitch_pub;
	ros::Publisher yaw_pub;
	ros::Publisher roll_pub;

	ros::Subscriber teacher_joints_sub;
	ros::Subscriber teacher_switch_sub;

	void joints_cb(const sensor_msgs::JointState::ConstPtr& states)
	{
		outer_yaw.data = states->position[0];
		shoulder_pitch.data = states->position[1];
		elbow_pitch.data = states->position[2];
		wrist_platform.data = states->position[3];
		wrist_pitch.data = states->position[4];
		wrist_yaw.data = states->position[5];
		wrist_roll.data = states->position[6];
	}

	void switch_cb(const std_msgs::Int32::ConstPtr& data)
	{
		switch_value = data->data;
	}

	float deg2rad(int deg)
	{
		float rad = 3.14159*deg/180;
		return rad;
	}

	int rad2deg(float rad)
	{
		int deg = 180*rad/3.14159;
		return deg;
	}	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mtm_student");
	mtm_student stud;
	try
		{
			stud.spin();
		}
	catch (ros::Exception& e)
		{
			cout<<"Shutting Down"<<endl;
		}
	cv::destroyAllWindows();
	return 0;
}



