#include <ros/ros.h>
#include <math.h>
#define _USE_MATH_DEFINES // for the PI constant
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <iomanip>
#include <string>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <stdlib.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace Eigen;
using namespace std;

robot_state::RobotStatePtr ur_kinematic_state;
robot_state::RobotStatePtr ur_kinematic_state_n_plus_1;
const robot_state::JointModelGroup *joint_model_group;

tf::TransformListener *listener;
tf::StampedTransform transform_aux;
Eigen::Affine3d ur5_base_link_to_ur5_ee_link;

Eigen::Affine3d baseToWrist3Link;
sensor_msgs::JointState currJointState;

Eigen::Vector3d p_n_plus_1;								 //Position of the end effector at n+1 time
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); //Jacobian reference position
Eigen::VectorXd q_n(6);
Eigen::VectorXd q_n_plus_1(6);
Eigen::MatrixXd jacobian(6, 6);
Eigen::MatrixXd jacobian_n_plus_1(6, 6);

JacobiSVD<MatrixXd> svdOfJ_n(3, 3);
JacobiSVD<MatrixXd> svdOfJ_n_plus_1(3, 3);
Eigen::Vector3d S_n, S_n_plus_1;

//Joint Limits
double degToRad = M_PI / 180.0;
double qMin[6] = {-86 * degToRad, -181 * degToRad, 12 * degToRad, -359 * degToRad, -359 * degToRad, -359 * degToRad};
double qMax[6] = {123 * degToRad, 359 * degToRad, 146 * degToRad, 359 * degToRad, 359 * degToRad, 359 * degToRad};

//Variables for safety plane
Eigen::Vector3d n1, p1;

void jointStateCallback(const sensor_msgs::JointStatePtr &msg)
{
	//Forward Kinematics
	currJointState = *msg;
	//ur_kinematic_state->setVariableValues(*msg); //Set the current joint state

	std::vector<double> joint_values;
	//Get the current joint position
	for (int i = 0; i < 6; i++)
	{
		q_n[i] = currJointState.position[i];
		joint_values.push_back(currJointState.position[i + 3]);
	}

	/*ur_kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	ur_kinematic_state->setVariablePosition("ur5_shoulder_pan_joint", currJointState.position[3]);
	ur_kinematic_state->setVariablePosition("ur5_shoulder_lift_joint", currJointState.position[4]);
	ur_kinematic_state->setVariablePosition("ur5_elbow_joint", currJointState.position[5]);
	ur_kinematic_state->setVariablePosition("ur5_wrist_1_joint", currJointState.position[6]);
	ur_kinematic_state->setVariablePosition("ur5_wrist_2_joint", currJointState.position[7]);
	ur_kinematic_state->setVariablePosition("ur5_wrist_3_joint", currJointState.position[8]);

	base_link_to_ur5_ee_link = ur_kinematic_state->getGlobalLinkTransform("ur5_ee_link");
	base_link_to_prism_plate = ur_kinematic_state->getGlobalLinkTransform("prism_plate");
	ur5_base_link_to_ur5_ee_link = base_link_to_prism_plate.inverse()*base_link_to_ur5_ee_link;

	//std::vector<double> joint_values;
	const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();	
	ur_kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
  		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}
	//Get the axis angle representation
	AngleAxisd rotVector(ur5_base_link_to_ur5_ee_link.rotation());
	Vector3d UR5rotVector = rotVector.axis() * rotVector.angle();*/

	//Use tf to find the ur5 forward kinematics
	try
	{
		ros::Time now = ros::Time::now();
		/*listener->waitForTransform("ur5_base", now,
								   "ur5_tool0", now,
								   "prism_plate", ros::Duration(0.2));
		listener->lookupTransform("ur5_base", now,
								  "ur5_tool0", now,
								  "prism_plate", transform_aux);
								  ros::Time now = ros::Time::now();*/
		listener->waitForTransform("base_footprint", now,
								   "prism_plate", now,
								   "base_link", ros::Duration(0.2));
		listener->lookupTransform("base_footprint", now,
								  "prism_plate", now,
								  "base_link", transform_aux);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}
	tf::transformTFToEigen(transform_aux, ur5_base_link_to_ur5_ee_link);
	
	//Get position and rotation vectors
	Vector3d UR5posVector(transform_aux.getOrigin());
	//Get the axis angle representation
	AngleAxisd rotVector(ur5_base_link_to_ur5_ee_link.rotation());
	Vector3d UR5rotVector = rotVector.axis() * rotVector.angle();	

	//Show the current pose
	cout << "Pos: " << endl
		 << UR5posVector << endl;
	//Get the rotation vector shown in UR5 interface
	cout << "Rotation vector: "
		 << "rx: " << UR5rotVector(0) << " "
		 << "ry: " << UR5rotVector(1) << " "
		 << "rz: " << UR5rotVector(2) << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_interface");

	ros::NodeHandle nh;

	listener = new tf::TransformListener(ros::Duration(10));
	ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, &jointStateCallback);

	/*Translation3d transGripper(Vector3d(0, 0, 0.17389));
	eeLinkToGripper.setIdentity();
	eeLinkToGripper = eeLinkToGripper * transGripper;*/

	// We will start by instantiating a
	// `RobotModelLoader`_
	// object, which will look up
	// the robot description on the ROS parameter server and construct a
	// :moveit_core:`RobotModel` for us to use.

	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	// Using the :moveit_core:`RobotModel`, we can construct a
	// :moveit_core:`RobotState` that maintains the configuration
	// of the robot. We will set all joints in the state to their
	// default values.
	robot_state::RobotStatePtr temp_kinematic_state(new robot_state::RobotState(kinematic_model));
	ur_kinematic_state = temp_kinematic_state;
	//ur_kinematic_state->setToDefaultValues();
	ur_kinematic_state->setVariablePosition("ur5_shoulder_pan_joint", 0.0);
	ur_kinematic_state->setVariablePosition("ur5_shoulder_lift_joint", -3.1416190306292933);
	ur_kinematic_state->setVariablePosition("ur5_elbow_joint", 1.5707863012896937);
	ur_kinematic_state->setVariablePosition("ur5_wrist_1_joint", -1.5707863012896937);
	ur_kinematic_state->setVariablePosition("ur5_wrist_2_joint", -1.5707863012896937);
	ur_kinematic_state->setVariablePosition("ur5_wrist_3_joint", 3.1416190306292933);

	joint_model_group = kinematic_model->getJointModelGroup("ur5_arm");
	std::vector<double> joint_values;
	const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
	ur_kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	//Set the loop rate
	ros::Rate loop_rate(30);

	//Read the joint states once at the beggining
	currJointState = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states"));

	while (ros::ok())
	{

		ros::spinOnce();   //Check for callbacks
		loop_rate.sleep(); //Sleep until getting to the given rate
	}

	return 0;
}