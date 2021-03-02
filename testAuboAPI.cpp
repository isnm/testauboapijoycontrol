

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


double currentjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double targetjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree

 

int state = 0;
/**
bool forwardKinematic(std::vector<double> js, std::vector<double> &xyzrpy)
{	
	aubo_msgs::GetFK get_fk;
	get_fk.request.joint[0] = currentjoint[0];
	get_fk.request.joint[1] = currentjoint[1];
	get_fk.request.joint[2] = currentjoint[2];
	get_fk.request.joint[3] = currentjoint[3];
	get_fk.request.joint[4] = currentjoint[4];
	get_fk.request.joint[5] = currentjoint[5];
	if (client_fk.call(get_fk))
	{
		get_fk.response.pos[0];
		return true
	}
	else
		return false
}
/**
bool getforward(std::vector<double> jointstate, std::vector<double> &xyzrpy) {
    if(FK_client.call())
	{
		xyzrpy[0] = get_fk.response.pos[0];
		xyzrpy[1] = msg.response.pos[1];
		xyzrpy[2] = msg.response.pos[2];
		orientation : quatanion to rpy
		xyzrpy[3] = ???;
       		
	}
    else
        return false;
}
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
	state = 0;
      if(ret && state == 1)
      {
        ROS_INFO("state 1");
        for(int i=0; i<6; i++) {
          targetjoint[i] = currentjoint[i];         
            }  
        targetjoint[0] = currentjoint[0]-0.1;	

        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 2){
        ROS_INFO("state 2");
           for(int i=0; i<6; i++) {
            targetjoint[i] = currentjoint[i];         
            } 
        targetjoint[0] = currentjoint[0]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 3)
      {
        ROS_INFO("state 3");
           for(int i=0; i<6; i++) {
        targetjoint[i] = currentjoint[i];         
    	} 
        targetjoint[1] = currentjoint[1]-0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 4)
      {
        ROS_INFO("state 4");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[1] = currentjoint[1]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 5)
      {
        ROS_INFO("state 5");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[2] = currentjoint[2]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 6)
      {
        ROS_INFO("state 6");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[2] = currentjoint[2]-0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 7)
      {
        ROS_INFO("state 7");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[4] = currentjoint[4]-0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 8)
      {
        ROS_INFO("state 8");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[4] = currentjoint[4]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 9)
      {
        ROS_INFO("state 9");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[3] = currentjoint[3]-0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 10)
      {
        ROS_INFO("state 10");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[3] = currentjoint[3]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 11)
      {
        ROS_INFO("state 11");
	aubo_msgs::GetFK get_fk;
	get_fk.request.joint.resize(6);
	get_fk.request.joint[0] = currentjoint[0];
	get_fk.request.joint[1] = currentjoint[1];
	get_fk.request.joint[2] = currentjoint[2];
	get_fk.request.joint[3] = currentjoint[3];
	get_fk.request.joint[4] = currentjoint[4];
	get_fk.request.joint[5] = currentjoint[5];
	double x=0.0,y=0.0,z=0.0 ;
	double q[4] = {0.0,0.0,0.0,0.0};
	
	if (FK_client.call(get_fk))
	{
		x= get_fk.response.pos[0]+0.1;
		y= get_fk.response.pos[1];
		z= get_fk.response.pos[2];
		q[0] = get_fk.response.ori[0];
		q[1] = get_fk.response.ori[1];
		q[2] = get_fk.response.ori[2];
		q[3] = get_fk.response.ori[3];
	
		
	}
	get_ik.request.ref_joint.resize(6);
	get_ik.request.ref_joint[0] = currentjoint[0];
	get_ik.request.ref_joint[1] = currentjoint[1];
	get_ik.request.ref_joint[2] = currentjoint[2];
	get_ik.request.ref_joint[3] = currentjoint[3];
	get_ik.request.ref_joint[4] = currentjoint[4];
	get_ik.request.ref_joint[5] = currentjoint[5];
    get_ik.request.pos.resize(3);
    get_ik.request.ori.resize(4);
    get_ik.request.pos[0] = x;
	get_ik.request.pos[1] = y;
	get_ik.request.pos[2] = z;
    get_ik.request.ori[0] = q[0]; 
	get_ik.request.ori[1] = q[1]; 
	get_ik.request.ori[2] = q[2]; 
	get_ik.request.ori[3] = q[3];
      	if (IK_client.call(get_ik))
      	{
        	targetjoint[0] = get_ik.response.joint[0];
		targetjoint[1] = get_ik.response.joint[1];
		targetjoint[2] = get_ik.response.joint[2];
		targetjoint[3] = get_ik.response.joint[3];
		targetjoint[4] = get_ik.response.joint[4];
		targetjoint[5] = get_ik.response.joint[5];
		
      	}
            
        targetjoint[5] = currentjoint[5]+0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 12)
      {
        ROS_INFO("state 12");
            for(int i=0; i<6; i++) {
                targetjoint[i] = currentjoint[i];
            }
        targetjoint[5] = currentjoint[5]-0.1;
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }

   
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
