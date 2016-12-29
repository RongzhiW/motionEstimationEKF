#ifndef VICON_H
#define VICON_H

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"

#include "uav_math.h"

using namespace std;

/**
  vicon pose is nwu
**/

//in nwu world frame
struct ViconPoint {
    double timeStamp;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
    double roll;
    double pitch;
    double yaw;

    double worldVx;
    double worldVy;
    double worldVz;

    double bodyAngX;
    double bodyAngY;
    double bodyAngZ;
    public:
        ViconPoint()
        {
            x=0;
            y=0;
            z=0;
            vx=0;
            vy=0;
            vz=0;
            ax=0;
            ay=0;
            az=0;
            roll=0;
            pitch=0;
            yaw=0;
            worldVx=0;
            worldVy=0;
            worldVz=0;
            bodyAngX=0;
            bodyAngY=0;
            bodyAngZ=0;

        }
        ViconPoint& operator = (const ViconPoint &that){
            this->x=that.x;
            this->y=that.y;
            this->z=that.z;
            this->vx=that.vx;
            this->vy=that.vy;
            this->vz=that.vz;
            this->ax=that.ax;
            this->ay=that.ay;
            this->az=that.az;
            this->roll=that.roll;
            this->pitch=that.pitch;
            this->yaw=that.yaw;
            this->worldVx=that.worldVx;
            this->worldVy=that.worldVy;
            this->worldVz=that.worldVz;
            this->bodyAngX=that.bodyAngX;
            this->bodyAngY=that.bodyAngY;
            this->bodyAngZ=that.bodyAngZ;
        }
};

class Vicon
{
public:
    Vicon(ros::NodeHandle *n):
    isConvertInit(false),
    startConvert(false),
    isposedata(false),
    isveldata(false),
    isViconUpdate(false),
    isMarkersUpdate(false),
    isFirstUpdateYaw(true),
      isViconAvaible(false),
    timeBetweenFrame(0.01){
        nh=*n;
        setup();
    }

    double dataTime,startTime;
    double frameCount,frameMarkersCount,frameCountStart,frameMarkersCountStart,frameCountLast;
    double timeBetweenFrame;
    ros::Subscriber viconData_sub,viconMarkers_sub;
    ViconPoint state,stateLast;

    double qbwStart[4],qWFromV[4];
    double Rwv[9];
    float translation[3],poseLast[3],velLast[3];
    geometry_msgs::TransformStamped viconPose;
    bool isConvertInit,startConvert,isposedata,isveldata,isViconUpdate,isMarkersUpdate,isFirstUpdateYaw;
    bool isViconAvaible;
    double viconStartYaw;
    //*****yaw based on initial frame***********
    double viconYawBasedInitFrame;
    //*****for filter gitch of vicon velocity and euler angle*****
    int vxGitchNum,vyGitchNum,vzGitchNum,rollGitchNum,pitchGitchNum,yawGitchNum;
    double viconQlast[4]={1,0,0,0};
    double qbv[4];


    void qtoEuler(double& roll,double& pitch,double& yaw,double q[]);

    void convert2NED(double w,double x,double y,double z);

    deque< pair<double, ViconPoint> > vicon_deque;

    uav_math *uavMathObj;
    //**********KF for vicon velocity**********
    float viconKF_vx,viconKF_vy,viconKF_vz;
    float kfP,kfQ,kfR,kfKk;
    //**********KF for vicon acceleration******
    float viconKF_ax,viconKF_ay,viconKF_az;
    float acckfP,acckfQ,acckfR,acckfKk;
    //*****************************************


private:
    ros::NodeHandle nh;

    void setup();
    void viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr &msg);
    void qmultiplyq(double q1[],double q2[],double q_result[]);
    void getRotation(double q[],double R[]);
    void transformWorldFromVicon(const double a_vicon[3],double a_world[3]);
};

#endif // Vicon_H
