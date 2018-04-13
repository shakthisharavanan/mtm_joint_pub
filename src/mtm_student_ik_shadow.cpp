#include <stdlib.h>
#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

using namespace cv;
using namespace std;

class mtm_student
{
	
private:

	int switch_value; 
	ros::NodeHandle n;

	vctDoubleVec mtm_joint_current;
	vctDoubleVec mtm_joint_command;
	vctFrm4x4 mtm_pose_current;
	geometry_msgs::Pose msg_pose_current;
	vctFrm4x4 mtm_pose_command;

	robManipulator mtm_manip;

	std_msgs::Float64 outer_yaw;
	std_msgs::Float64 shoulder_pitch;
	std_msgs::Float64 elbow_pitch;
	std_msgs::Float64 wrist_platform;
	std_msgs::Float64 wrist_pitch;
	std_msgs::Float64 wrist_yaw;
	std_msgs::Float64 wrist_roll;

	ros::Publisher pose_current_pub;
	ros::Publisher outer_yaw_pub;
	ros::Publisher shoulder_pub;
	ros::Publisher elbow_pub;
	ros::Publisher platform_pub;
	ros::Publisher pitch_pub;
	ros::Publisher yaw_pub;
	ros::Publisher roll_pub;

	ros::Subscriber student_cartesian_sub;
	ros::Subscriber student_joints_sub;
	ros::Subscriber teacher_switch_sub;

public:

mtm_student()
{
	switch_value = 0; 

	student_cartesian_sub = n.subscribe("/mtm_student_cartesian_pose_command", 100, &mtm_student::cartesian_pose_cb, this);
	student_joints_sub = n.subscribe<sensor_msgs::JointState>("/mtm_student/joint/states", 100, &mtm_student::joints_feedback_cb, this);
	teacher_switch_sub = n.subscribe<std_msgs::Int32>("/mtm_teacher_switch", 100, &mtm_student::switch_cb, this);

	pose_current_pub  = n.advertise<geometry_msgs::Pose>("/mtm_student_cartesian_pose_current", 100);
	outer_yaw_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_outer_yaw_joint/SetPositionTarget", 100);
	shoulder_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_shoulder_pitch_joint/SetPositionTarget", 100);
	elbow_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_elbow_pitch_joint/SetPositionTarget", 100);
	platform_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_platform_joint/SetPositionTarget", 100);
	pitch_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_pitch_joint/SetPositionTarget", 100);
	yaw_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_yaw_joint/SetPositionTarget", 100);
	roll_pub = n.advertise<std_msgs::Float64>("/mtm_student/right_wrist_roll_joint/SetPositionTarget", 100);

}

float deg2rad(int deg)
{
	float rad = 3.14159*deg/180.0;
	return rad;
}

int rad2deg(float rad)
{
	int deg = 180*rad/3.14159;
	return deg;
}

void joints_feedback_cb(const sensor_msgs::JointState::ConstPtr& states)
{
	mtm_joint_current[0] = states->position[0];   // outer_yaw_joint
    mtm_joint_current[1] = states->position[1];   // shoulder_pitch_joint
    mtm_joint_current[2] = states->position[3];   // elbow_pitch_joint
    mtm_joint_current[3] = states->position[4];   // wrist_platform_joint
    mtm_joint_current[4] = states->position[5];   // wrist_pitch_joint
    mtm_joint_current[5] = states->position[6];   // wrist_yaw_joint
    mtm_joint_current[6] = states->position[7];   // wrist_roll_joint
}

void cartesian_pose_cb(const geometry_msgs::Pose &pos)
{
	mtsROSToCISST(pos, mtm_pose_command);
}

void switch_cb(const std_msgs::Int32::ConstPtr& data)
{
	switch_value = data->data;
}
	
void publish_joints()
{

	outer_yaw.data      = mtm_joint_command[0];
	shoulder_pitch.data = mtm_joint_command[1];
	elbow_pitch.data    = mtm_joint_command[2];
	wrist_platform.data = mtm_joint_command[3];
	wrist_pitch.data    = mtm_joint_command[4];
	wrist_yaw.data      = mtm_joint_command[5];
	wrist_roll.data     = mtm_joint_command[6];

	outer_yaw_pub.publish(outer_yaw);
	shoulder_pub.publish(shoulder_pitch);
	elbow_pub.publish(elbow_pitch);
	platform_pub.publish(wrist_platform);
	pitch_pub.publish(wrist_pitch);
	yaw_pub.publish(wrist_yaw);
	roll_pub.publish(wrist_roll);
}

void publish_current_pose()
{
	mtm_pose_current = mtm_manip.ForwardKinematics(mtm_joint_current);
    mtsCISSTToROS(mtm_pose_current, msg_pose_current);
    pose_current_pub.publish(msg_pose_current);

}

void spin()
{	
	ros::Rate loop_rate(100);
	mtm_joint_command[0] = deg2rad(75);
    mtm_joint_command[1] = deg2rad(45);
    mtm_joint_command[2] = deg2rad(45);
    mtm_joint_command[3] = deg2rad(60);
    mtm_joint_command[4] = deg2rad(90);
    mtm_joint_command[5] = deg2rad(45);
    mtm_joint_command[6] = deg2rad(270);
    publish_joints();

	while (ros::ok())
	{
		ros::spinOnce();
		mtm_manip.InverseKinematics(mtm_joint_current, mtm_pose_command);
		mtm_joint_command = mtm_joint_current;
		publish_joints();
		publish_current_pose();
		loop_rate.sleep();
	}
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



