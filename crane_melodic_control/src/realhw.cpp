#include <ros/package.h>
#include <angles/angles.h>
#include <crane_melodic_control/realhw.h>
#include <iostream>
#include <math.h>



CranePlus::CranePlus()
{
    
    joint_names[0].data = "crane_plus_revolute_joint";
    joint_names[1].data = "crane_plus_flex_joint";
    joint_names[2].data = "crane_plus_elbow_joint";
    joint_names[3].data = "crane_plus_wrist_joint";
    joint_names[4].data = "crane_plus_moving_finger_joint";
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("crane_plus_shoulder_revolute_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("crane_plus_shoulder_flex_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface.registerHandle(state_handle_2);

    hardware_interface::JointStateHandle state_handle_3("crane_plus_elbow_joint", &pos_[2], &vel_[2], &eff_[2]);
    jnt_state_interface.registerHandle(state_handle_3);

    hardware_interface::JointStateHandle state_handle_4("crane_plus_wrist_joint", &pos_[3], &vel_[3], &eff_[3]);
    jnt_state_interface.registerHandle(state_handle_4);

    hardware_interface::JointStateHandle state_handle_5("crane_plus_moving_finger_joint", &pos_[4], &vel_[4], &eff_[4]);
    jnt_state_interface.registerHandle(state_handle_5);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("crane_plus_shoulder_revolute_joint"), &cmd_[0]);
    jnt_pos_interface.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("crane_plus_shoulder_flex_joint"), &cmd_[1]);
    jnt_pos_interface.registerHandle(pos_handle_2);

    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("crane_plus_elbow_joint"), &cmd_[2]);
    jnt_pos_interface.registerHandle(pos_handle_3);

    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("crane_plus_wrist_joint"), &cmd_[3]);
    jnt_pos_interface.registerHandle(pos_handle_4);

    hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("crane_plus_moving_finger_joint"), &cmd_[4]);
    jnt_pos_interface.registerHandle(pos_handle_5);

    registerInterface(&jnt_pos_interface);
    pub = nh.advertise<sensor_msgs::JointState>("request", 100000, true);
}

void CranePlus::timeCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg_sub)
{
    for(int i=0; i<5; i++) {
        for(int j=0; j<5; j++){
            if(joint_names[i].data == msg_sub->joint_names[j]){pos_[i] = msg_sub->points[0].positions[j]; }
        }
    }
}

void CranePlus::read(ros::Time time, ros::Duration period)
{
    sub = nh.subscribe("dynamixel_workbench/joint_trajectory", 1000, &CranePlus::timeCallback, this);
}

void CranePlus::write(ros::Time time, ros::Duration period)
{
    sensor_msgs::JointState msg_pub;
    msg_pub.name.resize(5);
    msg_pub.position.resize(5);
    for(int i=0; i<5; i++) {
        msg_pub.name[i] = joint_names[i].data;
        msg_pub.position[i] = cmd_[i];
        // ROS_INFO("cmd data : %.5f", cmd_[i]);
        }
    pub.publish(msg_pub);
}