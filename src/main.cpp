#include <iostream>

using namespace std;

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <sys/stat.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/PoseStamped.h"

#include "uav_motion_estimation_ekf.h"
#include "vicon.h"
#include "uav_sensors.h"

#define pi 3.1415927
#define focal_length 16/24*1000


int main(int argc, char** argv){

    ros::init(argc, argv, "UavMotionEstimationEKF");
    ros::NodeHandle node;


    Vicon *viconNode=new Vicon(&node);
    UavSensors *uavSensorsNode=new UavSensors(&node,viconNode);

    ROS_INFO("motionEstimationEKF start!");
    UavMotionEstimationEKF *motionEstiEKF=new UavMotionEstimationEKF(&node,viconNode,uavSensorsNode);
    motionEstiEKF->startMotionEstiLoop();
    motionEstiEKF->startSpinOnceLoop();

    while(1);
    return 0;
}



