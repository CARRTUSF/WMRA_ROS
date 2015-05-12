
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "MotorController.h"

using std::vector;

class wmraArm : public hardware_interface::RobotHW
{
public:
  wmraArm() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_0("wmra_arm_joint_0", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_0);

   hardware_interface::JointStateHandle state_handle_1("wmra_arm_joint_1", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_1);

   hardware_interface::JointStateHandle state_handle_2("wmra_arm_joint_2", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_2);

   hardware_interface::JointStateHandle state_handle_3("wmra_arm_joint_3", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_3);

   hardware_interface::JointStateHandle state_handle_4("wmra_arm_joint_4", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_4);

   hardware_interface::JointStateHandle state_handle_5("wmra_arm_joint_5", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_5);

   hardware_interface::JointStateHandle state_handle_6("wmra_arm_joint_6", &pos[6], &vel[6], &eff[6]);
   jnt_state_interface.registerHandle(state_handle_6);


   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_0(jnt_state_interface.getHandle("wmra_arm_joint_0"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_0);

   hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("wmra_arm_joint_1"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_1);

   hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("wmra_arm_joint_2"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_2);

   hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("wmra_arm_joint_3"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_3);

   hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("wmra_arm_joint_4"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_4);

   hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("wmra_arm_joint_5"), &cmd[5]);
   jnt_pos_interface.registerHandle(pos_handle_5);

   hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("wmra_arm_joint_6"), &cmd[6]);
   jnt_pos_interface.registerHandle(pos_handle_6);


   registerInterface(&jnt_pos_interface);

   robotIn.resize(4);
   robotOut.resize(4);

    // Starting Galil Motor Controller
    bool debugVal = false;
    galil.initialize(debugVal);
  }

  void read()
  {
    // Read actuator state from hardware
/*
	for(int i = 0; i<7; i++)
	{ 
	    robotIn = galil.readState(i);
	    pos[i] = robotIn[0];
	    vel[i] = robotIn[1];
	    eff[i] = robotIn[2];
	}
*/
  }
  
 vector<double> read2() 
 {
	return galil.readState();
 }
  
  void write()
  {
    // Send actuator command to hardware
/*
	for(int i = 0; i<7; i++)
	{ 
	    robotOut[0] = pos[i];
	    robotOut[1] = vel[i];
	    robotOut[2] = eff[i];
	    robotOut[3] = cmd[i];
	    galil.positionControl(i,robotOut[3]); // Motor 0, Send position
	}
*/
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];

  // Galil Controller
  MotorController galil;
  vector<double> robotIn;
  vector<double> robotOut;
};


int main(int argc, char** argv)
{
  // Initializing this ROS node
  ros::init(argc,argv, "WMRA_Hardware");

ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/wmra_joint_state/JS", 1);

sensor_msgs::JointState joint_state;

  wmraArm robot;
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Timing
  ros::Time prevTime = ros::Time::now();
  ros::Rate rate(1.0);
  
joint_state.name.resize(7);
joint_state.position.resize(7);
joint_state.name[0] ="wmra_arm_joint_0";
joint_state.name[1] ="wmra_arm_joint_1";
joint_state.name[2] ="wmra_arm_joint_2";
joint_state.name[3] ="wmra_arm_joint_3";
joint_state.name[4] ="wmra_arm_joint_4";
joint_state.name[5] ="wmra_arm_joint_5";
joint_state.name[6] ="wmra_arm_joint_6";

  while (ros::ok())
  {
     // Updating Timing
     const ros::Time curTime = ros::Time::now();
     const ros::Duration period = curTime - prevTime;

    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.position = robot.read2();

    joint_pub.publish(joint_state);
 //    robot.read();
     cm.update(curTime, period);
     robot.write(); 
 
     rate.sleep();
  }
  return 0;
}
