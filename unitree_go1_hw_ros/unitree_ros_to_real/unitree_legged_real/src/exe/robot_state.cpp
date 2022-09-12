#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <ros/ros.h>
#include "convert.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
        //   high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) //for sending command
        //   high_udp(8091, "192.168.12.1", 8083, sizeof(HighCmd), sizeof(HighState)) //for reciving over wirless
          high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)) //for reciving over wirless

    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;
ros::Subscriber sub_cmd_vel;


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

    custom.high_cmd = rosMsg2Cmd(msg);
    // ROS_INFO("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    // ROS_INFO("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    // ROS_INFO("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_high_state_sub");

    ros::NodeHandle nh;

    unitree_legged_msgs::HighState high_state_ros;

    ros::Publisher pub_high, pub_jointState, pub_IMU, pub_sdkOdom;
    tf2_ros::TransformBroadcaster br;

    ros::Publisher pub_footContact_FL, pub_footContact_FR, pub_footContact_RR, pub_footContact_RL;

    sub_cmd_vel = nh.subscribe("/cmd_vel", 1, cmdVelCallback);


    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    pub_jointState = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_IMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
    pub_sdkOdom = nh.advertise<nav_msgs::Odometry>("odom_legSdk", 1);

    pub_footContact_FL = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/FL", 1);
    pub_footContact_FR = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/FR", 1);
    pub_footContact_RL = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/RL", 1);
    pub_footContact_RR = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/RR", 1);

    LoopFunc loop_udpSend("high_udp_send", 0.001, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.001, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spinOnce();
    
    ros::Time currentTime;


    ros::Rate r(1000); // 1000 hz
    while (ros::ok())
    {
        currentTime = ros::Time::now();
        high_state_ros = state2rosMsg(custom.high_state);

        pub_high.publish(high_state_ros);


        // publish IMU data:
        sensor_msgs::Imu imuMsg;
        imuMsg.header.stamp = currentTime;
        imuMsg.header.frame_id = "imu_link";

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], high_state_ros.imu.rpy[2]);
        geometry_msgs::Quaternion quat_msg;
        imuMsg.orientation = tf2::toMsg(quat_tf);

        imuMsg.angular_velocity.x = high_state_ros.imu.gyroscope[0];
        imuMsg.angular_velocity.y = high_state_ros.imu.gyroscope[1];
        imuMsg.angular_velocity.z = high_state_ros.imu.gyroscope[2];

        imuMsg.linear_acceleration.x = high_state_ros.imu.accelerometer[0];
        imuMsg.linear_acceleration.y = high_state_ros.imu.accelerometer[1];
        imuMsg.linear_acceleration.z = high_state_ros.imu.accelerometer[2];

        pub_IMU.publish(imuMsg);


        // SDK Odom data:
        nav_msgs::Odometry legSdkOdomMsg;
        legSdkOdomMsg.header.frame_id ="odom";
        legSdkOdomMsg.child_frame_id = "base";
        legSdkOdomMsg.header.stamp = currentTime;
        legSdkOdomMsg.pose.pose.position.x = high_state_ros.position[0];
        legSdkOdomMsg.pose.pose.position.y = high_state_ros.position[1];
        legSdkOdomMsg.pose.pose.position.z = high_state_ros.position[2];
        legSdkOdomMsg.pose.pose.orientation = tf2::toMsg(quat_tf);

        pub_sdkOdom.publish(legSdkOdomMsg);

        // tf transform of odom:
        geometry_msgs::TransformStamped transformStampedMsg;
        transformStampedMsg.header.stamp = currentTime;
        transformStampedMsg.header.frame_id = "odom";
        transformStampedMsg.child_frame_id = "base";

        transformStampedMsg.transform.translation.x = high_state_ros.position[0];
        transformStampedMsg.transform.translation.y = high_state_ros.position[1];
        transformStampedMsg.transform.translation.z = high_state_ros.position[2];

        transformStampedMsg.transform.rotation.x = quat_tf.x();
        transformStampedMsg.transform.rotation.y = quat_tf.y();
        transformStampedMsg.transform.rotation.z = quat_tf.z();
        transformStampedMsg.transform.rotation.w = quat_tf.w();
        // br.sendTransform(transformStampedMsg);


        // foot contact:
        geometry_msgs::WrenchStamped contact_FL, contact_FR, contact_RL, contact_RR; 

        contact_FL.header.stamp = currentTime;
        contact_FL.header.frame_id = "FL_foot";
        contact_FR.header.stamp = currentTime;
        contact_FR.header.frame_id = "FR_foot";
        contact_RL.header.stamp = currentTime;
        contact_RL.header.frame_id = "RL_foot";
        contact_RR.header.stamp = currentTime;
        contact_RR.header.frame_id = "RR_foot";

        contact_FL.wrench.force.z = high_state_ros.footForce[FL_];
        contact_FR.wrench.force.z = high_state_ros.footForce[FR_];
        contact_RL.wrench.force.z = high_state_ros.footForce[RL_];
        contact_RR.wrench.force.z = high_state_ros.footForce[RR_];
        
        pub_footContact_FL.publish(contact_FL);
        pub_footContact_FR.publish(contact_FR);
        pub_footContact_RL.publish(contact_RL);
        pub_footContact_RR.publish(contact_RR);

        // publish joint states:
        sensor_msgs::JointState go1JointState;
        go1JointState.header.stamp = currentTime;

        go1JointState.name.push_back("FR_hip_joint");
        go1JointState.name.push_back("FR_thigh_joint");
        go1JointState.name.push_back("FR_calf_joint");
        go1JointState.position.push_back(high_state_ros.motorState[FR_0].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FR_0].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FR_0].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[FR_1].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FR_1].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FR_1].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[FR_2].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FR_2].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FR_2].tauEst);

        go1JointState.name.push_back("FL_hip_joint");
        go1JointState.name.push_back("FL_thigh_joint");
        go1JointState.name.push_back("FL_calf_joint");
        go1JointState.position.push_back(high_state_ros.motorState[FL_0].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FL_0].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FL_0].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[FL_1].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FL_1].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FL_1].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[FL_2].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[FL_2].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[FL_2].tauEst);

        go1JointState.name.push_back("RR_hip_joint");
        go1JointState.name.push_back("RR_thigh_joint");
        go1JointState.name.push_back("RR_calf_joint");
        go1JointState.position.push_back(high_state_ros.motorState[RR_0].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RR_0].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RR_0].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[RR_1].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RR_1].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RR_1].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[RR_2].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RR_2].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RR_2].tauEst);

        go1JointState.name.push_back("RL_hip_joint");
        go1JointState.name.push_back("RL_thigh_joint");
        go1JointState.name.push_back("RL_calf_joint");
        go1JointState.position.push_back(high_state_ros.motorState[RL_0].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RL_0].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RL_0].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[RL_1].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RL_1].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RL_1].tauEst);
        go1JointState.position.push_back(high_state_ros.motorState[RL_2].q);
        go1JointState.velocity.push_back(high_state_ros.motorState[RL_2].dq);
        go1JointState.effort.push_back(high_state_ros.motorState[RL_2].tauEst);
    

        pub_jointState.publish(go1JointState);

        ros::spinOnce(); //spin once to allow subscriber to be processed
        r.sleep();
    }



    return 0;
}