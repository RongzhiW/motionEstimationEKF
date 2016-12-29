#ifndef UAV_SENSORS_H
#define UAV_SENSORS_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include  "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"

#include "motionEstimationEKF/optical_flow_rad.h"
#include "motionEstimationEKF/velocityMsg.h"

#include "vicon.h"
#include "uav_math.h"

//#include "ardrone_autonomy/Navdata.h"

struct OpticalFlowData{
    //optical flow global path and velocity
    float opticalGlobal_x_m;
    float opticalGlobal_y_m;
    float opticalGlobal_z_m;
    float opticalGlobal_vx;
    float opticalGlobal_vy;
    float opticalGlobal_vz;
    float opticalLocal_vx;
    float opticalLocal_vy;
    float opticalLocal_vz;
    float distance;
    float distanceRaw;
    int quality;
    uint32_t timeStamp;
public:
    OpticalFlowData()
    {
        opticalGlobal_x_m=0;
        opticalGlobal_y_m=0;
        opticalGlobal_z_m=0;
        opticalGlobal_vx=0;
        opticalGlobal_vy=0;
        opticalGlobal_vz=0;
        opticalLocal_vx=0;
        opticalLocal_vy=0;
        opticalLocal_vz=0;
        distance=0;
        distanceRaw=0;
        quality=0;
        timeStamp=0;
    }
    OpticalFlowData& operator = (const OpticalFlowData &that){
        this->opticalGlobal_x_m=that.opticalGlobal_x_m;
        this->opticalGlobal_y_m=that.opticalGlobal_y_m;
        this->opticalGlobal_z_m=that.opticalGlobal_z_m;
        this->opticalGlobal_vx=that.opticalGlobal_vx;
        this->opticalGlobal_vy=that.opticalGlobal_vy;
        this->opticalGlobal_vz=that.opticalGlobal_vz;
        this->opticalLocal_vx=that.opticalLocal_vx;
        this->opticalLocal_vy=that.opticalLocal_vy;
        this->opticalLocal_vz=that.opticalLocal_vz;
        this->distance=that.distance;
        this->distance=that.distanceRaw;
        this->quality=that.quality;
        this->timeStamp=that.timeStamp;
    }
};

struct ImuData{

    //onboard gyrometer value in NED
    double roll, pitch, yaw;
    //quadnion
    double w,x,y,z;
    //onboard accelerameter value in NED
    double accelera_x, accelera_y, accelera_z;
    //a simple kf filter for accelerameter in NED
    double acceleraRaw_x,acceleraRaw_y,acceleraRaw_z;
    //onboard angular velocity in NED
    double angularVx,angularVy,angularVz;
    double timeStamp;
};

class UavSensors
{

public:
    //constructor
    UavSensors(ros::NodeHandle *n, Vicon *viconTmp){
        nh=*n;
        viconObj=viconTmp;
        setup();
    }

    ~UavSensors(void){}

    ImuData imuData,imuDataLast;
    double imuYawRaw;
    OpticalFlowData opticalFlowData;
    double R11,R12,R13,R21,R22,R23,R31,R32,R33;
    double qObj0[4],qObj1[4];
    double qNED_NWU[4];
    bool isImuUpdated,isImuAvaible;
    bool isOpticalFlowAvaible,isBaroAvaible,isMagAvaible;
    bool isFirstUpdate;
    int flowUpdateCount;
    double FirstYaw;
    Vicon *viconObj;
    uav_math *uavMathObj;
    //for ekf
    deque< pair<double, ImuData> > imuDeque;
    double startTime;

    double ardroneBaro, ardroneHeightFromBaro, ardroneBaroStart, ardroneHeightFromBaroLast;
    double uav_01Baro, uav_01HeightFromBaro, uav_01BaroStart, uav_01HeightFromBaroLast;
    geometry_msgs::Vector3 ardroneMag;
    geometry_msgs::Vector3 uav_01Mag;
    bool isBaroFirstUpdate;
    double accAxBias,accAyBias,accAzBias;
    int sampleNumForBias;
    double opticalFlowLastDis=0;
    double opticalFlowLastLastDis=0;
    int axGitchNum,ayGitchNum,azGitchNum;
    int distanceGitchNum;

    double qInit[4]={0.932697,0.0479,0.05667,-0.3529};
    double qInitConju[4]={0.932697,-0.0479,-0.05667,0.3529};
    double accInit[4]={0,-1.372,0.392,9.8};


    //***************kf for filter imu accelerometer data***********
        double imuAccKfP[3]={1,1,1};
        double imuAccKfR[3]={0.12,0.12,0.12};
        double imuAccKfQ[3]={0.01,0.01,0.01};
        double imuAccKfK[3]={0,0,0};
        double AccKfFilted[3];
    //******************************************************



private:
    ros::NodeHandle nh;
    ros::Subscriber px4flow_sub;
    ros::Subscriber imu_sub;
    //ros::Subscriber ardroneBaroSub;
    ros::Subscriber uav_01BaroSub;
    ros::Subscriber uav_01MagSub;

    bool setup();
    void flowCb(const motionEstimationEKF::optical_flow_rad &msg);
    void imuCb(const sensor_msgs::Imu &msg);
    void uav_01BaroCb(const sensor_msgs::FluidPressure &msg);
    void uav_01MagCb(const sensor_msgs::MagneticField &msg);
};

#endif // UAV_SENSORS_H
