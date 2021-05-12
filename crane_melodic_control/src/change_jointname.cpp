#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "string"


class ChangeJointname
{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_joint_state;
    ros::Publisher pub_joint_state;
    sensor_msgs::JointState rejoint_state;
    std::string joint_rename[6];


public:
    ChangeJointname(){
        sub_joint_state = nh_.subscribe<sensor_msgs::JointState>("dynamixel_workbench/joint_states", 100, &ChangeJointname::JointState_callback, this);
        pub_joint_state = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1, this);
        joint_rename[0] = "crane_plus_shoulder_revolute_joint";
        joint_rename[1] = "crane_plus_shoulder_flex_joint";
        joint_rename[2] = "crane_plus_elbow_joint";
        joint_rename[3] = "crane_plus_wrist_joint";
        joint_rename[4] = "crane_plus_moving_finger_joint";
        joint_rename[5] = "crane_plus_gripper_joint";

        rejoint_state.name.clear();
        rejoint_state.position.clear();
        rejoint_state.velocity.clear();
        rejoint_state.effort.clear();
        rejoint_state.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; i++)
        {
            rejoint_state.name.push_back(joint_rename[i]);
            rejoint_state.position.push_back(0.0);
            rejoint_state.velocity.push_back(0.0);
            rejoint_state.effort.push_back(0.0);
        }
    }
    void JointState_publish(){
        pub_joint_state.publish(rejoint_state);
    }

private:
    void JointState_callback(const sensor_msgs::JointState::ConstPtr& jointstate){

        rejoint_state.name.clear();
        rejoint_state.position.clear();
        rejoint_state.velocity.clear();
        rejoint_state.effort.clear();
        rejoint_state.header.stamp = ros::Time::now();

        for(int i=0; i<jointstate->name.size(); i++){
            rejoint_state.name.push_back(joint_rename[i]);
            rejoint_state.position.push_back(jointstate->position[i]);
            rejoint_state.velocity.push_back(jointstate->velocity[i]);
            rejoint_state.effort.push_back(jointstate->effort[i]);

        }
        rejoint_state.name.push_back(joint_rename[5]);
        rejoint_state.position.push_back(0.0);
        rejoint_state.velocity.push_back(0.0);
        rejoint_state.effort.push_back(0.0);
    }


};


int main(int argc, char **argv){
    ros::init(argc, argv, "joint_rename_node");
    ChangeJointname ChangeJointname;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ChangeJointname.JointState_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    

    return 0;
}


