#include "ros/ros.h"
#include "ros/time.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <vector>

std_msgs::String joint_name[5];
std::vector< std::vector<double> > joint_pos(5, std::vector<double>(5));
std::vector< std::vector<double> > joint_vel(5, std::vector<double>(5));
std::vector< std::vector<double> > joint_acc(5, std::vector<double>(5));
std::vector< std::vector<double> > joint_eff(5, std::vector<double>(5));
// std_msgs::Float64 joint_vel[5];
// std_msgs::Float64 joint_eff[5];

//コールバックがあるとグローバルに読み込み
void monitorJointState_callback(const trajectory_msgs::JointTrajectory::ConstPtr& jointtrajectory)
{
  int i,j,k;
  joint_pos.resize(jointtrajectory->points.size());
  joint_vel.resize(jointtrajectory->points.size());
  joint_acc.resize(jointtrajectory->points.size());
  // joint_eff.resize(jointtrajectory->points.size());
  ROS_INFO("trajectory items:%d", jointtrajectory->points.size());
  
  for(k=0; k < jointtrajectory->points.size(); k++){
    // ROS_INFO("k:%d",k);
    joint_pos[k].resize(5);
    joint_vel[k].resize(5);
    joint_acc[k].resize(5);
    // joint_eff[k].resize(5);
    for(i=0;i<5;i++){
      for(j=0;j<5;j++){
        if(joint_name[i].data == jointtrajectory->joint_names[j]){
          // ROS_INFO("joint_pos:%d, %d", joint_pos.size(), joint_pos[k].size());
          
          joint_pos.at(k).at(i) = jointtrajectory->points[k].positions[j];    // ポジション読み出し
          joint_vel.at(k).at(i) = jointtrajectory->points[k].velocities[j];    // 速度読み出し
          
          joint_acc.at(k).at(i) = jointtrajectory->points[k].accelerations[j];    // 加速度読み出し
          // ROS_INFO("j:%.4f", jointtrajectory->points[k].effort[j]);
          // joint_eff.at(k).at(i) = jointtrajectory->points[k].effort[j];    // 負荷読み出し
          
          // joint_v[kel[i].data = jointstate->velocity[j];    // 速度読み出し
          // joint_eff[i].data = jointstate->effort[j];    // 負荷読み出し
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  int i;
  ros::init(argc, argv, "moveit2dynamixel");   // ノードの初期化
  ros::NodeHandle nh;                             // ノードハンドラ

  ros::Subscriber sub_joints;                     // サブスクライバの作成
  sub_joints = nh.subscribe("/joint_trajectory", 1, monitorJointState_callback);    // moveitから取る

  ros::Publisher arm_pub;                         //パブリッシャの作成
  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",100);

  ros::Rate loop_rate(45);  // 制御周期45Hz(サーボの更新が20msecなのですが、50Hzだと余裕ない感じ)

  trajectory_msgs::JointTrajectory jtp0;
  
  jtp0.header.frame_id = "base_link";   // ポーズ名（モーション名)

  jtp0.joint_names.resize(5); // 名前
  jtp0.joint_names[0] ="joint_1";
  jtp0.joint_names[1] ="joint_2";
  jtp0.joint_names[2] ="joint_3";
  jtp0.joint_names[3] ="joint_4";
  jtp0.joint_names[4] ="joint_5";
  
  joint_name[0].data = "crane_plus_shoulder_revolute_joint";
  joint_name[1].data = "crane_plus_shoulder_flex_joint";
  joint_name[2].data = "crane_plus_elbow_joint";
  joint_name[3].data = "crane_plus_wrist_joint";
  joint_name[4].data = "crane_plus_moving_finger_joint";


  //初期化
  jtp0.points.resize(1);

  jtp0.points[0].positions.resize(5);
  jtp0.points[0].velocities.resize(5);
  jtp0.points[0].accelerations.resize(5);
  jtp0.points[0].effort.resize(5);

  for(i=0;i<5;i++){
    jtp0.points[0].positions[i] = 0.0;
    jtp0.points[0].velocities[i] = 0.0;
    jtp0.points[0].accelerations[i] = 0.0;
    jtp0.points[0].effort[i] = 0.0;
  }

  jtp0.points[0].time_from_start = ros::Duration(0.2);  //実行時間20msec (10msecだと動作しない;;)
  int k=0;
  double data=0.0;

  while (ros::ok()) { // 無限ループ

    jtp0.header.stamp = ros::Time::now();
    jtp0.points.clear();
    jtp0.points.resize(3);
    for(i=0; i<joint_pos.size(); i++){
      for(k=0; k<3; k++){
        if(i<joint_pos.size()){
          jtp0.points[k].positions.resize(5);
          jtp0.points[k].velocities.resize(5);
          jtp0.points[k].accelerations.resize(5);
          jtp0.points[k].time_from_start = ros::Duration(0.2);
          for(int j=0; j<5; j++){
            jtp0.points[k].positions[j] = joint_pos[i][j];
            jtp0.points[k].velocities[j] = joint_vel[i][j];
            jtp0.points[k].accelerations[j] = joint_acc[i][j];
            // jtp0.points[0].effort[j] = joint_eff[i][j];
          }
        }
        else{
          break;
        }
        // ROS_INFO("%.4f, %.4f, %.4f, %.4f, %.4f",joint_pos.at(i).at(0), joint_pos.at(i).at(1), joint_pos.at(i).at(2), joint_pos.at(i).at(3), joint_pos.at(i).at(4));
        i++;
      }
      jtp0.points.resize(k);
      arm_pub.publish(jtp0);
      jtp0.points.clear();
      jtp0.points.resize(3);
      // ROS_INFO("PUBLISHED");


    loop_rate.sleep(); // 待ち
    }
    /**
    joint_pos.resize(3);
    joint_vel.resize(3);
    joint_acc.resize(3);
    // joint_eff.resize(1);
    **/
    for(k=0; k<joint_pos.size(); k++){
      for(i=0; i<5; i++){
        data = joint_pos[joint_pos.size()-1][i];
        joint_pos[k][i] = data;
        data = joint_vel[joint_pos.size()-1][i];
        joint_vel[k][i] = data;
        data = joint_acc[joint_pos.size()-1][i];
        joint_acc[k][i] = data;
        // joint_eff[0][i] = jtp0.points[0].effort[i];
      }
    }
    ros::spinOnce();   // コールバック関数を呼ぶ
    //ROS_INFO("torikometeru? = %f",joint_pos[1].data);

  }
  
  return 0;
}