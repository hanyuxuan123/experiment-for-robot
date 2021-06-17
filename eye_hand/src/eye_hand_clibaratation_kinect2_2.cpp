#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "handeye.h"
#include <opencv2/core/eigen.hpp> 
#include <iostream>
#include <fstream>
using namespace Eigen;
using namespace std;
double x,y,z;
double a1,a2,a3,a4,a5,a6,a7,a8,a9;


int main(int argc, char** argv){
  ros::init(argc, argv, "two_tf_transform");
  ros::NodeHandle node;

  	ofstream outfile_1("out_robot_kinect_azure.txt", ios::trunc);
    ofstream outfile_2("out_kinect_kinect_azure.txt", ios::trunc);

int i=0;
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  // vector<Eigen::Matrix4d> Hgij;
  // vector<Eigen::Matrix4d> Hcij;
    vector<cv::Mat> Hgij_1;
	vector<cv::Mat> Hcij_1;
  vector<cv::Mat> Hgij;
	vector<cv::Mat> Hcij;
//  * @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
//  * @param Hcij [in] 4x4xN matrix:The transformation between the cameras.

  while (node.ok()&&i<=18)  //设置监视哨，一共进行18个循环，循环结束就会跳出，结束整个node
  {
    tf::StampedTransform transform;
    tf::StampedTransform transform1;
  try
          {
        //得到坐标kinect2和坐标标定板之间的关系
             listener.waitForTransform("kinect2_2_rgb_optical_frame", "camera_marker", ros::Time(0), ros::Duration(3.0));
             listener.lookupTransform("kinect2_2_rgb_optical_frame", "camera_marker", ros::Time(0), transform);
            }
  catch (tf::TransformException &ex)
       {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

  try
      {
      //得到机器人link0与link8之间的关系
             listener.waitForTransform("panda_link0", "panda_link8", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("panda_link0", "panda_link8", ros::Time(0), transform1);
            //可以通过修改来实现不同机器人之间的转换
       }
  catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
    }
    
        x=transform.getOrigin().x();
        y=transform.getOrigin().y();
        z=transform.getOrigin().z();
    //两种不同的表示方法，来表示getRotation
        float robot_oriation_x=transform.getRotation().getX();
        float robot_oriation_y=transform.getRotation().getY();
        float robot_oriation_z=transform.getRotation().getZ();
        float robot_oriation_w=transform.getRotation().getW();
        cout<<x<<","<<y<<","<<z<<","<<robot_oriation_x<<","<<robot_oriation_y<<","<<robot_oriation_z<<","<<robot_oriation_w<<";"<<endl;
        outfile_2<<x<<","<<y<<","<<z<<","<<robot_oriation_x<<","<<robot_oriation_y<<","<<robot_oriation_z<<","<<robot_oriation_w<<";"<<endl;

         float robot_pose_x=transform1.getOrigin().x();
         float robot_pose_y=transform1.getOrigin().y();
         float robot_pose_z=transform1.getOrigin().z();
    //两种不同的表示方法，来表示getRotation
        float robot_oriation_x1=transform1.getRotation().getX();
        float robot_oriation_y1=transform1.getRotation().getY();
        float robot_oriation_z1=transform1.getRotation().getZ();
        float robot_oriation_w1=transform1.getRotation().getW();
    cout<<robot_pose_x<<","<<robot_pose_y<<","<<robot_pose_z<<","<<robot_oriation_x1<<","<<robot_oriation_y1<<","<<robot_oriation_z1<<","<<robot_oriation_w1<<";"<<endl;
		outfile_1  <<robot_pose_x<<","<<robot_pose_y<<","<<robot_pose_z<<","<<robot_oriation_x1<<","<<robot_oriation_y1<<","<<robot_oriation_z1<<","<<robot_oriation_w1<<";"<<endl;
//构造机器人基坐标系和机器人末端坐标系之间的关系矩阵
        Eigen::Quaterniond q_3(robot_oriation_w1,robot_oriation_x1,robot_oriation_y1,robot_oriation_z1);
        q_3.normalize();
        Eigen::Matrix3d R_3 = q_3.toRotationMatrix();
        Eigen::Vector3d t_3 = Eigen::Vector3d(robot_pose_x,robot_pose_y,robot_pose_z);
        Eigen::Matrix4d Trans_EndToBase;
        Trans_EndToBase.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        Trans_EndToBase.block<3,3>(0,0) = R_3;
        Trans_EndToBase.block<3,1>(0,3) = t_3;

        cv::Mat tempMat_1;
        eigen2cv(Trans_EndToBase, tempMat_1);


        Hgij_1.push_back(tempMat_1);

//构造相机坐标系和机器人末端坐标系之间的关系矩阵
        Eigen::Quaterniond q_2(robot_oriation_w,robot_oriation_x,robot_oriation_y,robot_oriation_z);
        q_2.normalize();
        Eigen::Matrix3d R_2 = q_2.toRotationMatrix();

        Eigen::Vector3d t_2 = Eigen::Vector3d(x, y, z);
        Eigen::Matrix4d Trans_CamToEnd; // Your Transformation Matrix
        Trans_CamToEnd.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        Trans_CamToEnd.block<3,3>(0,0) = R_2;
        Trans_CamToEnd.block<3,1>(0,3) = t_2;
        cv::Mat tempMat_2;
        eigen2cv(Trans_CamToEnd, tempMat_2);
        Hcij_1.push_back(tempMat_2);


        // cout<<"camtoend_1"<<endl<<Trans_CamToEnd<<endl;
        // cout<<"endtobase_1"<<endl<<Trans_EndToBase<<endl;
        // cout<<"camtoend"<<endl<<tempMat_2<<endl;
        // cout<<"endtobase"<<endl<<tempMat_1<<endl;
     sleep(10); //延迟10s发送一次
        i ++;  //监视哨一共会循环19次，选择多次循环以提高最后手眼标定的效果
    rate.sleep();
  }
  	outfile_1.close();
    outfile_2.close();
cv::Mat tempMat_3;
cv::Mat tempMat_4;
cv::Mat tempMat_5;
cv::Mat tempMat_6;
  for( int i = 0; i <17; i++ )
   {
      invert(Hgij_1[i+1], tempMat_3, DECOMP_LU);
      invert(Hcij_1[i+1], tempMat_4, DECOMP_LU);
      tempMat_5=tempMat_3*Hgij_1[i];
      tempMat_6=tempMat_4*Hcij_1[i];
      Hgij.push_back(tempMat_5);
      Hcij.push_back(tempMat_6);
   }
cv::Mat Hcg1(4, 4, CV_64FC1);
Tsai_HandEye(Hcg1,  Hcij, Hgij);
	Mat Hcg2(4, 4, CV_64FC1);
	DualQ_HandEye(Hcg2, Hgij, Hcij);

	Mat Hcg3(4, 4, CV_64FC1);
	Inria_HandEye(Hcg3, Hgij, Hcij);

	Mat Hcg4(4, 4, CV_64FC1);
	Navy_HandEye(Hcg4, Hgij, Hcij);

	Mat Hcg5(4, 4, CV_64FC1);
	Kron_HandEye(Hcg5, Hgij, Hcij);

	cout << Hcg1 << endl << endl;
	cout << Hcg2 << endl << endl;
	cout << Hcg3 << endl << endl;
	cout << Hcg4 << endl << endl;
	cout << Hcg5 << endl << endl;
  return 0;
};
 
