/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

// All the possible dependencies

#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk/dji_sdk_node.h"

#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

// Things fot making the BatterySensor Work, for now doesn't

#include <sensor_msgs/BatteryState.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

// This is for comands and photos reading

#include<iostream>
#include<fstream>
#include<unistd.h>

// All for editing and transporting photos

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <stdio.h>

#include <vector>
#include <cmath>

// From Guidance 

#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//#include <dji_sdk/shared_variabls.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "dji_sdk/LandingData.h"
#include "dji_sdk/LandingActivation.h"

using namespace cv_bridge;
#define WIDTH 320
#define HEIGHT 240

using namespace std;
using namespace cv;


// Defining variables for changing the coordinate system

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

int counterGuidance = 0;
float height = 15;	

// Arrays used when changing mission of the drone

double *homePoint = (double *) malloc(3*sizeof(double)); // saved home point

// For Landing
float *landingXY = (float *) malloc(3*sizeof(float)); // saved home point

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawRatePub;

ros::Publisher guidanceActivation;
ros::Subscriber landing_activation_sub;

bool start_decent = false;

std_msgs::Bool start_guidance;
bool landingActivation = false;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;

Mission patrol_mission;


void activation_callback(const dji_sdk::LandingActivation::ConstPtr& msg)
{
	landingActivation = msg->data;
}

int main(int argc, char** argv)
{
   
  ros::init(argc, argv, "demo_landing_node");
  ros::NodeHandle nh1;

  //ros::Duration(5.0).sleep();

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh1.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh1.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh1.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh1.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);

  ros::Subscriber obstacle_distance_sub = nh1.subscribe("/guidance/obstacle_distance", 10, obstacle_distance_callback);
  ros::Subscriber landing_data = nh1.subscribe("dji_sdk/landingData", 10, landing_data_callback);

  // Publish the control signal
  ctrlPosYawPub = nh1.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlVelYawRatePub = nh1.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  nh1.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

  guidanceActivation = nh1.advertise<std_msgs::Bool>("/guidance/guidanceActivation", 10);
 
  // Basic services
  sdk_ctrl_authority_service = nh1.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh1.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh1.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  landing_activation_sub   = nh1.subscribe("/dji_sdk/landingActivation", 10, activation_callback);


  if(landingActivation) {
  
    patrol_mission.reset();
    patrol_mission.start_gps_location = current_gps;

    homePoint[0] = current_gps.latitude;
    homePoint[1] = current_gps.longitude;
    homePoint[2] = landingXY[2];

    patrol_mission.setTarget(0, 0, 10, 0);
    patrol_mission.state = 1;
    ROS_INFO("##### Go to target height and start guidance");
   
    ros::spin();
}

  else {

  ros::Duration(2.0).sleep();
  ROS_INFO("Waiting for activation");
}
  
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{
    landingXY[2] = g_oa.ranges[0];
}

void landing_data_callback(const dji_sdk::LandingData::ConstPtr& msg) // Problems?
{
  start_decent = msg->activation_landing;
  landingXY[0] = msg->x;
  landingXY[1] = msg->y;
}



void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

// This is one step in mission 

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset; // how much the position has changed
  float speedFactorX, speedFactorY, speedFactor = 0.5; // This defines the speed 
  float yawThresholdInDeg   = 5;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  //double zOffsetRemaining = target_offset_z - current_gps.altitude;
  double zOffsetRemaining = target_offset_z - (landingXY[2] - homePoint[2]); // Problem with defining the starting position for height, approximately solves the problem

// Gives xy speeds so that it moves uniformly from A to B

  speedFactorX = (xOffsetRemaining>0) ? (xOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(yOffsetRemaining,2))) * speedFactor : -1 * (xOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(yOffsetRemaining,2))) * speedFactor;
  speedFactorY = (yOffsetRemaining>0) ? (yOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(yOffsetRemaining,2))) * speedFactor : -1 * (yOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(yOffsetRemaining,2))) * speedFactor; 

// Variables for rotating

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 10) // To print data
	  {
	    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
	    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);

}

// Speed distribution depending on position

  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactorX : -1 * speedFactorX;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactorY : -1 * speedFactorY;
  else
    yCmd = yOffsetRemaining;
 
    zCmd = target_offset_z;

    
  

if(info_counter > 10)
  {
    info_counter = 0;
    ROS_INFO("zCmd=%f, altitude=%f, z=%f, dz=%f", zCmd, current_gps.altitude, current_gps.altitude-homePoint[2], zOffsetRemaining);
  }


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 5)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    ctrlVelYawRatePub.publish(controlVelYawRate);


    break_counter++;
    return;  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_GROUND   |
                    DJISDK::STABLE_ENABLE);

    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    controlPosYaw.axes.push_back(flag);
    ctrlPosYawPub.publish(controlPosYaw);

    
  }

  if (std::abs(xOffsetRemaining) < 0.2 &&
      std::abs(yOffsetRemaining) < 0.2 &&
      std::abs(zOffsetRemaining) < 0.2 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 5)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool go_to_home(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}



// Here we define how the mission proceedes

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {

    start_time = ros::Time::now();

	if(!patrol_mission.finished)
	{
		patrol_mission.step();
		
	}


	else
	{
		while(guidanceActivation.getNumSubscribers()==0)
				{
	   				ROS_ERROR("Waiting for subscibers for guidanceActivation");
	   				sleep(10);
				}
		ROS_ERROR("Got subscriber");

		start_guidance.data = true;
    		guidanceActivation.publish(start_guidance);

		ros::Duration(5.0).sleep();
		
		if(start_decent) {

		counterGuidance = counterGuidance+1;
				
		ROS_INFO("M100  LANDING"); 		
		patrol_mission.reset();  	
		patrol_mission.start_gps_location = current_gps; 

		float decent = 0;
		if(landingXY[2] > 30) decent=3;
		else if(landingXY[2] > 20) decent=2;
		else if(landingXY[2] > 10) decent=1;
		else if(landingXY[2] > 5) decent=0.5;
		else decent=0.2;

		height =  height - decent;

		patrol_mission.setTarget(-landingXY[0], -landingXY[1], height, 0);
                patrol_mission.state = counterGuidance;
 
		ROS_INFO("##### Landing part %d ....", counterGuidance);
		ROS_INFO("##### Landing x = %f ....", -landingXY[0]);
		ROS_INFO("##### Landing y = %f ....", -landingXY[1]);


		}

		else {
		ros::Duration(2.0).sleep();
		ROS_INFO("Waiting for landingData");
		}
			

	
	}
   }
	
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}





