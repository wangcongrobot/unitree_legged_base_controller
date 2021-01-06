/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "aliengo_sdk/aliengo_sdk.hpp"
#include "convert.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

// using namespace UNITREE_LEGGED_SDK;
unitree_legged_msgs::HighCmd SendHighROS;
unitree_legged_msgs::HighState RecvHighROS;


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

void control_callback(const geometry_msgs::Twist& cmd_vel)
{
    
	ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);

	// for spot model.	
	SendHighROS.forwardSpeed = cmd_vel.linear.x;
	SendHighROS.rotateSpeed = -cmd_vel.angular.z;
}


template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl;
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    ros::init(argc, argv, "unitree_base_controller");
    ros::NodeHandle n;
    // ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    
    
    SendHighROS.forwardSpeed = 0.0f;
	SendHighROS.sideSpeed = 0.0f;
	SendHighROS.rotateSpeed = 0.0f;
	SendHighROS.forwardSpeed = 0.0f;

	SendHighROS.mode = 0;
	SendHighROS.roll  = 0;
	SendHighROS.pitch = 0;
	SendHighROS.yaw = 0;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, control_callback);

    ros::Rate loop_rate(500);
    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

	while (ros::ok())
	{
		roslcm.Get(RecvHighLCM); // receive the A1's message.
		//memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
		//printf("%f\n",  RecvHighROS.forwardSpeed);	

        // Just give a simple example: receive the velocity.
		if(SendHighROS.forwardSpeed != 0 || SendHighROS.rotateSpeed !=0)
		{
			SendHighROS.mode = 2;
		}
		else
		{
			SendHighROS.mode =1;
		}
        //memcpy(&SendHighLCM, &SendHighROS, sizeof(HighCmd));
		// SendHighLCM = ToLcm(SendHighROS);
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    if(firmwork == "3_1"){
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    }
    else if(firmwork == "3_2"){
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    }
    
}