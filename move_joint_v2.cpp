
/**
\file    move_joint_node.cpp
\brief  Joint mover using key strokes as command
 *
 *  This node reads input from the keyboard and moves a joint of the Baxter robot,
 *  incrementing/decrementing its position according to the key that was hit.
 *  The name of the joint which is controlled is set using a ROS parameter.
\author  Gaetan Garcia
\date    24/03/2021
*/

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <algorithm>

//ROS
#include "ros/ros.h"

//ROS msgs
#include <std_msgs/Char.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>

#define PLUS  (int(43)) //The ASCII code for the increment key
#define MINUS (int(45)) //The ASCII code for the decrement key
#define DEFAULT_INCR (double(5*M_PI/180.0))  /* 5 degrees */

//Namespaces
using namespace std;


/**
  \fn void jointStateCallback(sensor_msgs::JointState msg_state)
  \brief Captures the current joint state of the robot and stores it in a global variable to be accesible by the main.
  \note  In a real life application, you would want to make sure that your callback regularly updates the state.
  */

sensor_msgs::JointState last_state ;  //Variable to store the last known joint state of the robot
bool state_received = false ;
void jointStateCallback(sensor_msgs::JointState state_msg){
    // Because the Baxter has a gripper, there are joint state messages that concer only the gripper.
    // We filter them out here as they do not contain the arm information.
    if( state_msg.name.size() < 3 return ;
    last_state = state_msg ;
    state_received = true ;
}

/**
  \fn void keyHitCallback(std_msgs::Int16 msg_key_hit)
  \brief Captures the key hit and stores it for use by main program.
  */

bool new_key_received  = false ;
int  last_key ;

void keyHitCallback(std_msgs::Int16 msg_key_hit){
    new_key_received = true             ;
    last_key         = msg_key_hit.data ;  
}

int main (int argc, char** argv){

    //ROS Initialization
    ros::init(argc, argv, "move_joint_node");
    ROS_INFO("Node move_joint_node connected to ROS master.") ;
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.

    string joint_name ; //To store the name of the joint to move
    int incr_key, decr_key ;//The acii key codes to increment and decrement
    double joint_incr ;//The increment value

    // Getting the joint name from a parameter
    if( !nh_.getParam("joint_name",joint_name) ){
        // No joint name given: makes no sense to proceed.
        ROS_FATAL("Couldn't find parameter: joint_name\n");
        return ;
    } else {
        ROS_INFO("Controlling joint: %s\n",joint_name.c_str()) ;
    }

    // The next two parameters have default values.

    // Beware: the retrieved variables must be declared as C++ types, not ROS types...
    nh_.param("incr_key", incr_key, PLUS);
    ROS_INFO("Increment key: %d\n",incr_key) ;

    nh_.param("decr_key", decr_key, MINUS);
    ROS_INFO("Decrement key: %d\n",decr_key) ;

    nh_.param("joint_incr", joint_incr, DEFAULT_INCR);
    ROS_INFO("Joint increment: %f\n",joint_incr) ;

    // Subscribe
    ros::Subscriber baxter_joint_state = nh_.subscribe<sensor_msgs::JointState> ("/robot/joint_states"  , 1, jointStateCallback);
    ros::Subscriber key_hit = nh_.subscribe<std_msgs::Int16> ("/key_hit"  , 1, keyHitCallback);

    // Advertize
    ros::Publisher pub_command = nh_.advertise<baxter_core_msgs::JointCommand>("/joint_command", 1);

    ros::Rate rate(100);
    //ROS_INFO("SPINNING @ 100Hz");
    while (ros::ok()){
        ros::spinOnce();

        // Nothing makes sense if robot state has not been received yet...
        if( !state_received ) {
            ROS_INFO("Waiting for robot state...\n") ;
            rate.sleep() ;
            continue ;
        }

        // Now check that the joint name makes sense, otherwise exit loop and program.
        std::vector<string>::iterator itr = std::find(last_state.name.begin(), last_state.name.end(), joint_name);
        if (itr == last_state.name.cend()) {
            // The joint name is not recognized. Makes no sense to proceed.
            ROS_FATAL("Unknown joint name\n") ;
            break ;
        }
        int joint_index = std::distance(last_state.name.begin(), itr);

        // Nothing to do if no new keystroke has been received.
        if( !new_key_received ){
            rate.sleep() ;
            continue ;
        }
        // Last keystroke is going to be used right now. Prepare to detect that next one is new.
        new_key_received = false ;

        // Proceed to sending the joint command.
        double joint_value ;
        switch(last_key){
            case incr_key: joint_value = last_state.position[joint_index] + joint_incr ; ROS_INFO("Moving up.\n")  ; break ;
            case decr_key: joint_value = last_state.position[joint_index] - joint_incr ; ROS_INFO("Moving down.\n"); break ;
            default:       joint_value = last_state.position[joint_index] ;              ROS_INFO("Key ignored.\n");
        }
        baxter_core_msgs::JointCommand command_msg ;
        command_msg.mode = command_msg.POSITION_MODE ; 
        command_msg.names.push_back(joint_name) ;
        command.msg.position.push_back(joint_value) ;
        pub_command.publish( msg_command ) ;

        rate.sleep();
    }

    ROS_INFO("ROS-Node Terminated\n");
}

