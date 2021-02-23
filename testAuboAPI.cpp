/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <aubo_msgs/JointPos.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;
#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 20.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    0.02                // unit m/s


double currentjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double targetjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double zeropos[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
 

int state = 0;
int prev_state = state;
bool bContinue;
/**
bool forwardKinematic(std::vector<double> jointState, std::vector<double> &xyzrpy){
	if()
}

bool f4(std::vector<double> js, std::vector<double> &xyzrpy) {
    if(FK_client.call())
        xyzrpy[0] = msg.response.pos[0];
        xyzrpy[1] = msg.response.pos[1];
        xyzrpy[2] = msg.response.pos[2];
	orientation : quatanion to rpy
        xyzrpy[3] = ???;
        return true;
    else
        return false;
}

std::vector<double> js={0.0,0.0 ...};
std::vector<double> cart;
if( f4(js, cart) )
    x = cart[0];
**/
void jointCallback(const sensor_msgs::JointState msg){
     ROS_INFO("%f,%f,%f,%f,%f,%f", msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5]);
     for(int i=0; i<6; i++) {
        currentjoint[i] = msg.position[i];
             
    }
}
void joyCallback(const sensor_msgs::Joy msg){
    ROS_INFO("joyCallback : %s", msg.header.frame_id.c_str());

    //ปุ่มอนาล็อคซ้าย ขยับซ้าย << หมายถึงปุ่มไม่ใช่หุ่น (ขยับฐาน)
    if(msg.axes[0] > 0.25 && state == 0){
        state = 1;
    }
     //ปุ่มอนาล็อคซ้าย ขยับขวา ขยับฐาน
    else if (msg.axes[0] < -0.25 && state == 0){
        state = 2;
    }
    //ปุ่มอนาล็อคซ้ายขยับขึ้น joint2 ขยับขึ้น
    else if (msg.axes[1] > 0.25 && state == 0){

        state = 3;    
    }
    //ปุ่มอนาล็อคซ้ายขยับลง joint2 ขยับลง
    else if (msg.axes[1] < -0.25 && state == 0){

        state = 4; 
    }
    //ปุ่มอนาล็อคขวาขยับขึ้น joint3 ขยับขึ้น
    else if  (msg.axes[4] > 0.25 && state == 0){

        state = 5;
    }
    //ปุ่มอนาล็อคขวาขยับลง joint3 ขยับลง
    else if (msg.axes[4] < -0.25 && state == 0){

        state = 6;
    }
    //ปุ่มอนาล็อคขวาขยับ ขวา joint5 ขยับขวา
    else if (msg.axes[3] > 0.25 && state == 0){

        state = 7;    
    }
    //ปุ่มอนาล็อคขวาขยับ ซ้าย joint 5 ขยับซ้าย
    else if (msg.axes[3] < -0.25 && state == 0){

        state = 8;
    }
    //ปุ่มลูกศรขึ้นjoint4 ขยับขึ้น
    else if (msg.axes[7] == 1 && state == 0){

        state = 9;    
    }
    //ปุ่มลูกศรลงjoint4ขยับลง
    else if (msg.axes[7] == -1 && state == 0){

        state = 10;    
    }
    //ปุ่มลูกศรขวา joint6ขยับขวา
    else if (msg.axes[6] == 1 && state == 0){

        state = 11;
    }
    //ปุ่มลูกศรซ้าย joint6 ขยับซ้าย
    else if (msg.axes[6] == -1 && state == 0){

        state = 12;
    }
    else if (msg.buttons[0] == 1 && state ==0){
        state = 13;
    }
    else{
        state = 0;
    }

}


int main(int argc, char **argv)
{
 
  
  
  ros::init(argc, argv, "testAuboAPI");

  ros::NodeHandle n;

  ros::Subscriber joy_sub = n.subscribe("joy", 1000, joyCallback );   //subscribe joy
  ros::Subscriber jointState = n.subscribe("/joint_states",100, jointCallback); //subscribe jointstate

  AuboDriver robot_driver;
  bool ret = robot_driver.connectToRobotController();
  

  ros::ServiceClient FK_client = n.serviceClient<aubo_msgs::GetFK>("/aubo_driver/get_fk");  //client subscribe service FK` (Forward Kinematic)
  ros::ServiceClient IK_client = n.serviceClient<aubo_msgs::GetIK>("/aubo_driver/gest_ik");
  aubo_msgs::GetFK get_fk;
  aubo_msgs::GetIK get_ik;

  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                             6        /*collision class*/,
                                             true     /* Is allowed to read robot pose*/,
                                             true,    /*default */
                                             1000,    /*default */
                                             result); /*initialize*/

  ros::Rate loop_rate(10);
  while (ros::ok()){
      if(prev_state ==0 && state!=0){
            ROS_INFO("prev_state == 0");
            bContinue = true;
            prev_state=state;
      }

      if(bContinue){
            ROS_INFO("%d",robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveContinue));
            robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveContinue); //Continue = 2

      
      }


      
        for(int i=0; i<6; i++) {
          targetjoint[i] = currentjoint[i];         
            } 
      if(ret && state == 1)
      {
        ROS_INFO("state 1");

 
        targetjoint[0] = currentjoint[0]-1.0;	

        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
        
      }
      else if(ret && state == 2){
        ROS_INFO("state 2");

        targetjoint[0] = currentjoint[0]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 3)
      {
        ROS_INFO("state 3");

        targetjoint[1] = currentjoint[1]-1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);


      }
      else if(ret && state == 4)
      {
        ROS_INFO("state 4");

        targetjoint[1] = currentjoint[1]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 5)
      {
        ROS_INFO("state 5");

        targetjoint[2] = currentjoint[2]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 6)
      {
        ROS_INFO("state 6");

        targetjoint[2] = currentjoint[2]-1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 7)
      {
        ROS_INFO("state 7");

        targetjoint[4] = currentjoint[4]-1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 8)
      {
        ROS_INFO("state 8");

        targetjoint[4] = currentjoint[4]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 9)
      {
        ROS_INFO("state 9");

        targetjoint[3] = currentjoint[3]-1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 10)
      {
        ROS_INFO("state 10");

        targetjoint[3] = currentjoint[3]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 11)
      {
        ROS_INFO("state 11");

        targetjoint[5] = currentjoint[5]+1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 12)
      {

        ROS_INFO("state 12");

        targetjoint[5] = currentjoint[5]-1.0;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 13)
      {
        ROS_INFO("State 13");
        robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMoveStop); //RobotMoveStop =0
      }
 /**     else
      {
        ROS_INFO("MOVE PAUSE");
        robot_driver.robot_send_service_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand::RobotMovePause);
      }
**/
   
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
