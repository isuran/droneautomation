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

#include <iostream>
#include <vector>
#include <cmath>

// From Guidance 

#include <string.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Subscriber depth_image_sub;
ros::Subscriber imu_sub;
ros::Subscriber velocity_sub;
ros::Subscriber obstacle_distance_sub;
ros::Subscriber ultrasonic_sub;
ros::Subscriber position_sub;

// Matching

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

//#include "sys/stats.h" // Stats structure definition
//#include "sys/utils.h" // Drawing and printing functions

const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 100; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames

using namespace cv_bridge;
#define WIDTH 320
#define HEIGHT 240

using namespace std;
using namespace cv;

// Defining variables for changing the coordinate system

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
const int numPoints = 1; // number of way points +1

// Reset counters that govern the moving of the drone

//int reset1 = 0; // reset first time going back 
//int reset2 = 0; // reset going forward after going back 
int reset3 = 0; // reset when going home

// Matrices for soring data for the mission 

//double *waypoinMissionData = (double *) malloc(4*(numPoints)*sizeof(double)); // from file
//double *inputData = (double *) malloc(3*(numPoints)*sizeof(double)); // to use in mission 

// Arrays used when changing mission of the drone

//double *returnData = (double *) malloc(3*sizeof(double)); // to go back home
double *newCoordinateResume = (double *) malloc(3*sizeof(double)); // to resume mission
double *newCoordinateReturn = (double *) malloc(3*sizeof(double)); // to backtrack the mission
double *homePoint = (double *) malloc(3*sizeof(double)); // saved home point

// For Landing
double *landingXY = (double *) malloc(2*sizeof(double)); // saved home point

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawRatePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
sensor_msgs::BatteryState current_battery;

cv::Mat imgCameraLeft;
cv::Mat imgCameraRight;
	
Mission patrol_mission;

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imgCameraLeft = cv_ptr->image;
    cv::imshow("left_image", cv_ptr->image);
    cv::waitKey(1);
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imgCameraRight = cv_ptr->image;
    cv::imshow("right_image", cv_ptr->image);
    cv::waitKey(1);
}

/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
    cv_ptr->image.convertTo(depth8, CV_8UC1);
    //cv::imshow("depth_image", depth8);
    cv::waitKey(1);
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{
    //printf( "frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    //printf( "imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{
    //printf( "frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    //printf( "velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
}

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{
    //printf( "frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
    //printf( "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
}

/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{
    //printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    //for (int i = 0; i < 5; i++) {}
        //printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
}

/* motion */
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
    //printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
    //for (int i = 0; i < 5; i++) {}
        //printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}


int main(int argc, char** argv)
{
    usleep(3000000);
   
    ros::init(argc, argv, "demo_flight_control_node");
    ros::NodeHandle nh;

    // Subscribe to messages from dji_sdk_node
    ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
    ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
    ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
    ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
    //ros::Subscriber batteryState = nh.subscribe("dji_sdk/battery_state", 10, &battery_state_callback);

    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

    left_image_sub        = my_node.subscribe("/guidance/left_image",  10, left_image_callback);
    right_image_sub       = my_node.subscribe("/guidance/right_image", 10, right_image_callback);
    depth_image_sub       = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
    imu_sub               = my_node.subscribe("/guidance/imu", 1, imu_callback);
    velocity_sub          = my_node.subscribe("/guidance/velocity", 1, velocity_callback);
    obstacle_distance_sub = my_node.subscribe("/guidance/obstacle_distance", 1, obstacle_distance_callback);
    ultrasonic_sub = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
    position_sub = my_node.subscribe("/guidance/position", 1, position_callback);

    // Publish the control signal
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

    // Basic services
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  /*if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  } */


  if(obtain_control_result)
  {
    patrol_mission.reset();
    patrol_mission.start_gps_location = current_gps;

  //waypoinMissionData[0] = current_gps.latitude;
  //waypoinMissionData[1] = current_gps.longitude;
  //waypoinMissionData[2] = 40;
  //waypoinMissionData[4] = 0;

  homePoint[0] = current_gps.latitude;
  homePoint[1] = current_gps.longitude;
  homePoint[2] = current_gps.altitude;

  // Reads from file the mission and saves in waypointMissionData | latitude | longitude | altitude | YAW |

  //ifstream file("/home/ivica/catkin_ws/src/Onboard-SDK-ROS-3.6/dji_sdk_demo/src/demo_flight_control_mission.txt");
  //string line;
  //int i = 1, j;

    //while(std::getline(file,line)){
    //std::istringstream iss(line);
    //float val;
    //j = 0;
    //while(iss >> val){
        //waypoinMissionData[4*i+j] = val;
	//ROS_INFO("data %f ", waypoinMissionData[4*i+j]);
        //j++;
    //}
    //i++;
  //}

  //for(int k=0; k<numPoints; k++) {
	// First point difference from home point express in xyz coordinates
	//if(k==0) {
		//inputData[k] = (waypoinMissionData[1] - current_gps.longitude) * deg2rad * C_EARTH * cos(waypoinMissionData[0]*deg2rad);
		//inputData[k+1] = (waypoinMissionData[0] - current_gps.latitude) * deg2rad * C_EARTH;
		//inputData[k+2] = waypoinMissionData[3];
		//ROS_INFO("data %f, %f, %f", inputData[k], inputData[k+1], inputData[k+2]);

	//}
	// Difference between to points in xyz coordinates
	//else{
		//inputData[3*k] = (waypoinMissionData[4*k+1] - waypoinMissionData[4*(k-1)+1]) * deg2rad * C_EARTH * cos(waypoinMissionData[0]*deg2rad);
		//inputData[3*k+1] = (waypoinMissionData[4*k] - waypoinMissionData[4*(k-1)]) * deg2rad * C_EARTH;
		//inputData[3*k+2] = (waypoinMissionData[4*k+3]>180) ? waypoinMissionData[4*k+3]-360 : waypoinMissionData[4*k+3];
		//ROS_INFO("data %f, %f, %f", inputData[3*k], inputData[3*k+1], inputData[3*k+2]);
	//}	
  //}

    patrol_mission.setTarget(0, 0, 40, 0);
    patrol_mission.state = 1;
    ROS_INFO("##### Start route %d ....", patrol_mission.state);
  

  ros::spin();

  }
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/


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

  float speedFactorX, speedFactorY, speedFactor = 1; // This defines the speed 
  float yawThresholdInDeg   = 5;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  //double zOffsetRemaining = target_offset_z - current_gps.altitude;
  double zOffsetRemaining = target_offset_z - (current_gps.altitude - homePoint[2]); // Problem with defining the starting position for height, approximately solves the problem

// Gives xy speeds so that it moves uniformly from A to B

  speedFactorX = (xOffsetRemaining>0) ? (xOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(xOffsetRemaining,2))) * speedFactor : -1 * (xOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(xOffsetRemaining,2))) * speedFactor;
  speedFactorY = (yOffsetRemaining>0) ? (yOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(xOffsetRemaining,2))) * speedFactor : -1 * (yOffsetRemaining/sqrt(pow(xOffsetRemaining,2)+pow(xOffsetRemaining,2))) * speedFactor; 

// Variables for rotating

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25) // To print data
	  {
	    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
	    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
	   // ROS_INFO("Battery level is %g", current_battery.percentage);

	// Coordinates for resuming
	     
	  newCoordinateResume[0] = xOffsetRemaining; 
	  newCoordinateResume[1] = yOffsetRemaining;

	// Coordinates for returning
	  
	  newCoordinateReturn[0] = localOffset.x;
	  newCoordinateReturn[1] = localOffset.y;

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
  

if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("zCmd=%f, altitude=%f, z=%f, dz=%f", zCmd, current_gps.altitude, current_gps.altitude-homePoint[2], zOffsetRemaining);
  }


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 15)
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
    return;
  }
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

  if (std::abs(xOffsetRemaining) < 1.0 &&
      std::abs(yOffsetRemaining) < 1.0 &&
      std::abs(zOffsetRemaining) < 1.0 &&
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

  if (inbound_counter > 10)
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

/*void battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& msg) // Problems?
{
  current_battery = *msg;
}*/

// Here we define how the mission proceedes

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {

// Reads comands from file

    //ifstream file2("/home/ivica/catkin_ws/src/Onboard-SDK-ROS-3.6/dji_sdk_demo/src/komande.txt");
    //int val;
    //file2 >> val;

    start_time = ros::Time::now();
    // here enter read from file and than if options for stop, back, resume and go home
	
	/*if(val == 1 && patrol_mission.state < numPoints){ // To procede normaly with the mission val = 1

	    for(int j=1; j<numPoints; j++) {
	    
		 if(patrol_mission.state == j) {

			if(!patrol_mission.finished)
			{
			  patrol_mission.step();
			}
			else
			{
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  patrol_mission.setTarget(inputData[j*3], inputData[1+j*3], 40, inputData[2+j*3]); // Loads new point
			  patrol_mission.state = j+1;
			  ROS_INFO("##### Start route %d ....", patrol_mission.state);
			  reset1 = 0;
			  reset2 = 0;
			}
	      	}
	    }
	}

	else if(val == 2 && patrol_mission.state < numPoints) { // To pause the mission
                  int j = patrol_mission.state;
		  patrol_mission.reset();
		  patrol_mission.start_gps_location = current_gps;
		  patrol_mission.setTarget(0, 0, 40, 0);
		  ROS_INFO("##### Pause");
		  patrol_mission.step();
		  reset1 = 0;
		  reset2 = 0;
	}

	else if(val == 3 && patrol_mission.state < numPoints) { // To retrace the mission

	    for(int j=0; j<numPoints; j++)    {

	    	if(patrol_mission.state == j) {

			if(reset1 == 0) { // Retrace the first half root part
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  newCoordinateReturn[2] = inputData[2+(j-1)*3]; // Angle of return 
			  ROS_INFO("##### Difference: %f , %f ", newCoordinateReturn[0], newCoordinateReturn[1]);
			  patrol_mission.setTarget(-newCoordinateReturn[0], -newCoordinateReturn[1], 40, newCoordinateReturn[2]);
			  patrol_mission.state = j;
			  ROS_INFO("##### Go back with route %d ... ", patrol_mission.state);
			  reset1 = 1; // change the reset counter because difference of defining return coordinates for next step
	 		}

			else if(!patrol_mission.finished)
			{
			  patrol_mission.step();
			}

			else if(patrol_mission.finished && patrol_mission.state <= 2) // If it gets to the second point it will stop
			{
		          reset2 = 0;
			  patrol_mission.state = numPoints;
			}

			else // Defines the rest of the retrace
			{
		          reset2 = 0;
			  reset3 = 0;
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  patrol_mission.setTarget(-inputData[(j-2)*3], -inputData[1+(j-2)*3], 40, inputData[2+(j-2)*3]);
			  patrol_mission.state = j-1;
			  ROS_INFO("##### Return with route %d ....", patrol_mission.state);
			}
	      	}
	    }
	}

	else if(val == 4 && patrol_mission.state < numPoints) { // To resume the mission

	     for(int j=1; j<numPoints; j++)    {

	    	 if(patrol_mission.state == j) {

			if(reset2 == 0 && reset1 == 0) { // Normal or Paused
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  newCoordinateResume[2] = inputData[2+(j)*3];
			  ROS_INFO("##### Difference: %f , %f ", newCoordinateResume[0], newCoordinateResume[1]);
			  patrol_mission.setTarget(newCoordinateResume[0], newCoordinateResume[1], 40, newCoordinateResume[2]);
			  patrol_mission.state = j;
			  ROS_INFO("##### Continue route %d ....", patrol_mission.state);
			  reset2 = 1;
			}

			if(reset2 == 0 && reset1 == 1) { // Returning
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  newCoordinateResume[2] = inputData[2+(j)*3];
			  ROS_INFO("##### Difference: %f , %f ", newCoordinateReturn[0], newCoordinateReturn[1]);
			  patrol_mission.setTarget(-newCoordinateReturn[0], -newCoordinateReturn[1], 40, newCoordinateReturn[2]);
			  patrol_mission.state = j;
			  ROS_INFO("##### Continue route %d ....", patrol_mission.state);
			  reset2 = 1;
			}

			else if(!patrol_mission.finished)
			{
			  patrol_mission.step();
			}

			else // When reset2 = 1 counter is active, this is normal mission
			{
			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
			  patrol_mission.setTarget(inputData[j*3], inputData[1+j*3], 40, inputData[2+j*3]);
			  patrol_mission.state = j+1;
			  ROS_INFO("##### Start route %d ....", patrol_mission.state);
			  reset1 = 0;	
			}
	      	}
	    }
	}

	else if(val == 5 || reset3 == 0) { // To get closer to home point
		
		ROS_INFO("M100 going home"); 
		reset2 = 0;
		reset1 = 0;

		if(!patrol_mission.finished)
			{
			  patrol_mission.step();
			}
		else
			{
			  patrol_mission.reset();
		  	  patrol_mission.start_gps_location = current_gps; 

			  returnData[0] = (homePoint[1] - current_gps.longitude) * deg2rad * C_EARTH * cos(homePoint[0]*deg2rad);
			  returnData[1] = (homePoint[0] - current_gps.latitude) * deg2rad * C_EARTH;
			  returnData[2] = waypoinMissionData[3];

			  ROS_INFO("Return to home distance %f, %f, %f", returnData[0], returnData[1], returnData[2]); 

    			  patrol_mission.setTarget(returnData[0], returnData[1], 40, returnData[2]);
                          patrol_mission.state = numPoints;

			  if (abs(homePoint[1] - current_gps.longitude) + abs(homePoint[0] - current_gps.latitude) < 1) {
				reset3 = 1;
			  }

		          ROS_INFO("##### Last root %d ....", patrol_mission.state);
			}
	
	}*/

	//else if(val == 5 || reset3 > 0) { // To get closer to home point
		
		if(!patrol_mission.finished)
			{
			  patrol_mission.step();
			}
		else
			{

			  patrol_mission.reset();
			  patrol_mission.start_gps_location = current_gps;
 			
			Mat imgPattern = imread("/home/ivica/catkin_ws/src/Onboard-SDK-ROS-3.6/dji_sdk_demo/src/a1-landing.png", CV_LOAD_IMAGE_GRAYSCALE);

		int minMatches = 5;

		// Initiate ORB detector
   		 Ptr<FeatureDetector> detector = ORB::create();

		// find the keypoints and descriptors with ORB
		
    		Ptr<DescriptorExtractor> extractor = ORB::create();
		  
		// Compute keypoints and descriptor from the source image in advance

		  	vector<KeyPoint> keypointsPattern;
			Mat descriptorsPattern;

			detector->detect(imgPattern, keypointsPattern);
		  	extractor->compute(imgPattern, keypointsPattern, descriptorsPattern);

		// Compute keypoints and descriptor from the left camera input

		 	vector<KeyPoint> keypointsCameraLeft;
		   	Mat descriptorsCameraLeft;

		// Compute keypoints and descriptor from the right camera input

		 	vector<KeyPoint> keypointsCameraRight;
		   	Mat descriptorsCameraRight;

		// Detect keypoints

		    	detector->detect(imgCameraLeft, keypointsCameraLeft);
			extractor->compute(imgCameraLeft, keypointsCameraLeft, descriptorsCameraLeft);
			detector->detect(imgCameraRight, keypointsCameraRight);
			extractor->compute(imgCameraRight, keypointsCameraRight, descriptorsCameraRight);				

		// Matching descriptor vectors using FLANN matcher

		  	FlannBasedMatcher matcher;

		  	std::vector< DMatch > matchesLeft;
		  	matcher.match( descriptorsPattern, descriptorsCameraLeft, matchesLeft );

			std::vector< DMatch > matchesRight;
		  	matcher.match( descriptorsPattern, descriptorsCameraRight, matchesRight );

			double max_dist = 0; double min_dist = 100;

		// Quick calculation of max and min distances between keypoints
		  	for( int i = 0; i < descriptorsPattern.rows; i++ )
		  	{ 
				double distLeft = matchesLeft[i].distance;
		    		if( distLeft < min_dist ) min_dist = distLeft;
		    		if( distLeft > max_dist ) max_dist = distLeft;
				double distRight = matchesRight[i].distance;
		    		if( distRight < min_dist ) min_dist = distRight;
		    		if( distRight > max_dist ) max_dist = distRight;
		  	}


		// Find the "good" matches

			vector<cv::DMatch> good_matchesLeft;
			for (int i = 0; i < matchesLeft.size(); ++i)
			{
		    		const float ratio = 0.8; // As in Lowe's paper; can be tuned
		    		if (matchesLeft[i].distance < 3*min_dist)
		    		{
					good_matchesLeft.push_back(matchesLeft[i]);
		    		}
			}

			vector<cv::DMatch> good_matchesRight;
			for (int i = 0; i < matchesRight.size(); ++i)
			{
		    		const float ratio = 0.8; // As in Lowe's paper; can be tuned
		    		if (matchesRight[i].distance < 3*min_dist)
		    		{
					good_matchesRight.push_back(matchesRight[i]);
		    		}
			}

			Mat img_matchesLeft;
			Mat img_matchesRight;
		  	drawMatches( imgPattern, keypointsPattern, imgCameraLeft, keypointsCameraLeft,
			       good_matchesLeft, img_matchesLeft, Scalar::all(-1), Scalar::all(-1),
			       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			drawMatches( imgPattern, keypointsPattern, imgCameraRight, keypointsCameraRight,
			       good_matchesRight, img_matchesRight, Scalar::all(-1), Scalar::all(-1),
			       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		/* If enough matches are found, we extract the locations of matched keypoints in both the images. They are passed to find the perpective transformation. Once we get this 3x3 transformation matrix, we use it to transform the corners of queryImage to corresponding points in trainImage. */ 

			if(good_matchesLeft.size() > minMatches && good_matchesRight.size() > minMatches) 
			{

			// Localize the object

			  	std::vector<Point2f> patternLeft;
				std::vector<Point2f> patternRight;
			  	std::vector<Point2f> cameraLeft;
				std::vector<Point2f> cameraRight;

			  	for( int i = 0; i < good_matchesLeft.size(); i++ )
			  	{
			    		// Get the keypoints from the good matches
			    		patternLeft.push_back( keypointsPattern[ good_matchesLeft[i].queryIdx ].pt );
			    		cameraLeft.push_back( keypointsCameraLeft[ good_matchesLeft[i].trainIdx ].pt );
			  	}

				for( int i = 0; i < good_matchesRight.size(); i++ )
			  	{
			    		// Get the keypoints from the good matches
			    		patternRight.push_back( keypointsPattern[ good_matchesRight[i].queryIdx ].pt );
			    		cameraRight.push_back( keypointsCameraRight[ good_matchesRight[i].trainIdx ].pt );
			  	}

			  	Mat H_Left = findHomography( patternLeft, cameraLeft, CV_RANSAC );	
				Mat H_Right = findHomography( patternRight, cameraRight, CV_RANSAC );

			// Get the corners from the image_1 ( the object to be "detected" )

			  	std::vector<Point2f> pattern_corners(4);
			  	pattern_corners[0] = cvPoint(0,0); pattern_corners[1] = cvPoint( imgPattern.cols, 0 );
			  	pattern_corners[2] = cvPoint( imgPattern.cols, imgPattern.rows ); pattern_corners[3] = cvPoint( 0, imgPattern.rows );
			  	std::vector<Point2f> camera_cornersLeft(4);
				std::vector<Point2f> camera_cornersRight(4);

			  	perspectiveTransform( pattern_corners, camera_cornersLeft, H_Left);
				perspectiveTransform( pattern_corners, camera_cornersRight, H_Right);

			// Draw lines between the corners (the mapped object in the scene - image_2 )
			  	line( img_matchesLeft, camera_cornersLeft[0] + Point2f( imgPattern.cols, 0), camera_cornersLeft[1] + Point2f( imgPattern.cols, 0), Scalar(0, 255, 0), 4 );
			  	line( img_matchesLeft, camera_cornersLeft[1] + Point2f( imgPattern.cols, 0), camera_cornersLeft[2] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );
			  	line( img_matchesLeft, camera_cornersLeft[2] + Point2f( imgPattern.cols, 0), camera_cornersLeft[3] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );
			  	line( img_matchesLeft, camera_cornersLeft[3] + Point2f( imgPattern.cols, 0), camera_cornersLeft[0] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );

			// Show detected matches
			  	imshow( "Good Matches & Pattern detection", img_matchesLeft );

			// Draw lines between the corners (the mapped object in the scene - image_2 )
			  	line( img_matchesRight,camera_cornersRight[0] + Point2f( imgPattern.cols, 0), camera_cornersRight[1] + Point2f( imgPattern.cols, 0), Scalar(0, 255, 0), 4 );
			  	line( img_matchesRight, camera_cornersRight[1] + Point2f( imgPattern.cols, 0), camera_cornersRight[2] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );
			  	line( img_matchesRight, camera_cornersRight[2] + Point2f( imgPattern.cols, 0), camera_cornersRight[3] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );
			  	line( img_matchesRight, camera_cornersRight[3] + Point2f( imgPattern.cols, 0), camera_cornersRight[0] + Point2f( imgPattern.cols, 0), Scalar( 0, 255, 0), 4 );

			// Show detected matches
			  	imshow( "Good Matches & Pattern detection", img_matchesRight );

			}

			int c = 0;
			float sumXLeft = 0;
			float sumYLeft = 0;

			for( int i = 0; i < good_matchesLeft.size(); i++ )
			{
			  	// Sum over all x and y
			   	sumXLeft = sumXLeft + keypointsCameraLeft[ good_matchesLeft[i].trainIdx ].pt.x;
				sumYLeft = sumYLeft + keypointsCameraLeft[ good_matchesLeft[i].trainIdx ].pt.y;
				c++;
			}

			float 	avgXLeft = sumXLeft/c;
			float 	avgYLeft = sumYLeft/c;

			c = 0;
			float sumXRight = 0;
			float sumYRight = 0;

			for( int i = 0; i < good_matchesRight.size(); i++ )
			{
			  	// Sum over all x and y
			   	sumXRight = sumXRight + keypointsCameraRight[ good_matchesRight[i].trainIdx ].pt.x;
				sumYRight = sumYRight + keypointsCameraRight[ good_matchesRight[i].trainIdx ].pt.y;
				c++;
			}

			float 	avgXRight = sumXRight/c;
			float 	avgYRight = sumYRight/c;

		// Dimnesions of the input image from the camera
			float dim_x = 320;
			float dim_y = 240;

		// Scaling factor
			float KNOWN_DISTANCE = 0.45;
			float KNOWN_PIXEL = 106.83;

		// Calculate the x and y distance of the drone from the target
			landingXY[0] = (avgXLeft + avgXRight - (dim_x))/KNOWN_PIXEL;
			landingXY[1] = ((dim_y) - avgYLeft - avgYRight)/KNOWN_PIXEL;


			patrol_mission.setTarget(landingXY[0], landingXY[1], 40.0-5.0*reset3, 0);
                        patrol_mission.state++;
 
			ROS_INFO("##### Landing %d ....", reset3);
			reset3 = reset3 + 1;
			}
	
	} 
	
  //}
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
  {
    ROS_ERROR("Takeoff failed. Distance is %f", current_gps.altitude - home_altitude);
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}



