#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

class CranePlus : public hardware_interface::RobotHW
{
public:
    CranePlus();

    ros::NodeHandle nh;

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.02); } //0.01s

    void timeCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg_sub);

    ros::Publisher pub;
    ros::Subscriber sub;

    void read(ros::Time, ros::Duration);
    void write(ros::Time, ros::Duration);
    std_msgs::String joint_names[5];

protected:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double cmd_[5];
    double pos_[5];
    double vel_[5];
    double eff_[5];


};