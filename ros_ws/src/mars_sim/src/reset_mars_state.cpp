#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_mars_joint_states");
    ros::NodeHandle n;
    //ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/mars_joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    sensor_msgs::JointState joint_state;

    //tf::TransformBroadcaster br;
    //tf::Transform odom_trans;

    // update transform
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = .0762;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(9);
    joint_state.position.resize(9);
    joint_state.name[0] = "right_wheel";
    joint_state.position[0] = 0.0;
    joint_state.name[1] = "left_wheel";
    joint_state.position[1] = 0.0;
    joint_state.name[2] = "prismatic_joint";
    joint_state.position[2] = 0.0;
    joint_state.name[3] = "ur5_shoulder_pan_joint";
    joint_state.position[3] = 0.0;
    joint_state.name[4] = "ur5_shoulder_lift_joint";
    joint_state.position[4] = -3.1416190306292933;
    joint_state.name[5] = "ur5_elbow_joint";
    joint_state.position[5] = 2.2689690589904785;
    joint_state.name[6] = "ur5_wrist_1_joint";
    joint_state.position[6] = -2.2689269224749964;
    joint_state.name[7] = "ur5_wrist_2_joint";
    joint_state.position[7] = -1.5707863012896937;
    joint_state.name[8] = "ur5_wrist_3_joint";
    joint_state.position[8] = 3.1415679454803467;
   
    int count = 0;
    while(ros::ok()&&count<30)
    {
        //send the joint state and broadcast the transform
        broadcaster.sendTransform(odom_trans);        
        joint_pub.publish(joint_state);
        count++;
        loop_rate.sleep();
    }

    //Move the platform with a circle of radius=1m
    /*const double degree = M_PI / 180;
    // robot state
    double tilt = 0, prism_pos = 0.0, prism_inc=0.005, angle = 0, ur_shoulder_angle, ur_shoulder_inc = degree;
    double x_inc = 0.005, x_pos = 0.0;
    while (ros::ok()) {
        //update joint_state
        joint_state.name.resize(9);
        joint_state.position.resize(9);
        joint_state.name[0] ="right_wheel";
        joint_state.position[0] = 0.0;
        joint_state.name[1] ="left_wheel";
        joint_state.position[1] = 0.0;
        joint_state.name[2] ="prismatic_joint";
        joint_state.position[2] = prism_pos;
        //joint_state.name[3] ="ur5_shoulder_pan_joint";
        //joint_state.position[3] = 0.0;
        joint_state.name[3] ="ur5_shoulder_pan_joint";
        joint_state.position[3] = ur_shoulder_angle;
        joint_state.name[4] ="ur5_shoulder_lift_joint";
        joint_state.position[4] = -3.1416190306292933;
        joint_state.name[5] ="ur5_elbow_joint";
        joint_state.position[5] = 2.2689690589904785;
        joint_state.name[6] ="ur5_wrist_1_joint";
        joint_state.position[6] = -2.2689269224749964;
        joint_state.name[7] ="ur5_wrist_2_joint";
        joint_state.position[7] = -1.5707863012896937;
        joint_state.name[8] ="ur5_wrist_3_joint";
        joint_state.position[8] = 3.1415679454803467;

        // update transform
        // (moving in a circle with radius=2)
        //odom_trans.transform.translation.x = cos(angle)*1;
        //odom_trans.transform.translation.y = sin(angle)*1;
        //odom_trans.transform.translation.z = 0.0;
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        odom_trans.transform.translation.x = x_pos;
        odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

        //odom_trans.setOrigin(tf::Vector3(1.0*cos(angle), 1.0*sin(angle), 0.0) );
        //odom_trans.setRotation(tf::createQuaternionFromYaw(angle+M_PI/2));
        //br.sendTransform(tf::StampedTransform(odom_trans, ros::Time::now(),"odom","base_footprint"));


        //send the joint state and transform
        ros::Time time_stamp = ros::Time::now();
        joint_state.header.stamp = time_stamp;
        joint_pub.publish(joint_state);
        odom_trans.header.stamp = time_stamp;
        broadcaster.sendTransform(odom_trans);

        x_pos += x_inc;
        if (x_pos < -2.0 || x_pos > 2.0) x_inc *= -1;

        prism_pos += prism_inc;
        if (prism_pos < 0.0 || prism_pos > 0.4) prism_inc *= -1;

        ur_shoulder_angle += ur_shoulder_inc;
        if (ur_shoulder_angle < -1.5707 || ur_shoulder_angle > 1.5707) ur_shoulder_inc *= -1;
        
        // Create new robot state
        angle += degree/5;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }*/

    return 0;
}