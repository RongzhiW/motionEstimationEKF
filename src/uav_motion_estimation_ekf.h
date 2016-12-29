#ifndef UAV_MOTION_ESTIMATION_EKF
#define UAV_MOTION_ESTIMATION_EKF


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ros/callback_queue.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include  "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"

#include "vicon.h"
#include "uav_sensors.h"
#include "uav_math.h"

using namespace std;
using namespace Eigen;

//inertial frame: NED

static void* startMotionEstiThread(void *args);
static void* startSpinOnceThread(void *args);
class UavMotionEstimationEKF
{

public:
    //constructor
    UavMotionEstimationEKF(ros::NodeHandle *n, Vicon *viconTmp, UavSensors *sensorsTmp){
        nh=*n;
        viconObj=viconTmp;
        uavSensorsObj=sensorsTmp;
        setup();
    }

    ~UavMotionEstimationEKF(void){threadQuit=true;}

public:
    //all data is NED
    ImuData imuData;
    OpticalFlowData opticalFlowData;
    OpticalFlowData **OpticalFlowDatasArray;
    ViconPoint viconData;
    ViconPoint **viconPointArray;
    int arraySize;
    int drift;

    float R_NED[9];
    bool threadQuit;
    bool isFirstWrite;
    double bodyVel[3];
    double worldVel[3];
    double imuDataStamp,imuDataStampLast;

    geometry_msgs::Vector3 uav_01Mag;
    double calcuYawValue,estiYawValue2PI,estiYawValue,calcuYawValueStart;
    double heightFromBaro;
    double worldPoseX,worldPoseY,worldPoseZ;
    double viconStartYaw,viconStartPoseX,viconStartPoseY,viconStartPoseZ;
    bool isFirstCalcuYaw,isFirstUpdateData;
    double bodyVzFromBaro,bodyVzFromSonar;
    double dragForceParamX,dragForceParamY;

    //**************calculate esti_velocity based on optical_flow quality*************
    double flowQualInitial,measFlowVxNoiseCovInit,measFlowVyNoiseCovInit,measFlowDistNoiseCovInit;
    double measImuAxtNoiseCov,measImuAytNoiseCov;
    double measFlowNoiseCovUpdateInterval;
    double qualLowThresh,qualHighThresh;
    double velLowThresh;
    double averageQual,sumQual;
    int processCounter;

    //**************run mode and some parameters**************************************
    bool useImuRaw,useImuRollAndPitchKfFused,useImuYawfusedWithMag;
    bool useSonarRaw,useSonarKf,useSonarComp;

    //**************for debug**********************
    ros::Publisher opticalFlowPosePub;
    ros::Publisher opticalFlowBodyVelPub;
    ros::Publisher opticalFlowWorldVelPub;
    ros::Publisher viconPosePub;
    ros::Publisher viconBodyVelPub;
    ros::Publisher viconWorldVelPub;
    ros::Publisher viconRPYPub;
    ros::Publisher imuAccRawPub;
    ros::Publisher imuAccPub;
    ros::Publisher imuRPYRawPub;

//**************ekf part for state estimation*************
public:
    void predict(Vector3d gyro, Vector3d imuAcc,double t);
    void correct(Vector3d imuAcc, double t);
    void process(Vector3d gyro,Vector3d imuAcc,VectorXd& xdot, MatrixXd& A, MatrixXd& W);

    void measurement(VectorXd& zhat, Vector3d gyro, MatrixXd &H, MatrixXd &V);

    void correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R, MatrixXd V);
    void correct_measurement(double t);

    void getState(Vector2d& rp, Vector2d& gyroBias,Vector3d& bodyVel,double& height);
    double get_time(){return current_t;}

    bool processSensorData(double timeNow);
    void autoAdaptEstiVel();

    void publishEstiBodyVel();
    void publishEstiRPY();
    void publishEstiWorldVel();
    void publishHeightFromSonar();
    void publishEstiWolrdPose();

private:
    VectorXd x;//state
    MatrixXd P;//state covariance
    MatrixXd Q;//imu observation noise
    MatrixXd measurement_R;

    const Vector3d GRAVITY = Vector3d(0, 0, 9.8);
    const int n_state = 6;

    Vector3d imuAcc;
    Vector3d bodyAngularVel;

    double current_t,start_t;
    bool imuEmpty,initialized,testFlag,coutOnceFlag;
    double gyroBiasTuo;//gyro bias shift period

    ros::Publisher estiBodyVel_pub;
    ros::Publisher estiRPY_pub;
    ros::Publisher estiWorldVel_pub;
    ros::Publisher heightFromSonar_pub;
    ros::Publisher estiWorldPose_pub;

    //for changing run mode and some parameters
    ros::Subscriber areoEstiRunMode_sub;
    ros::Subscriber areoEstiDragForceParam_sub;
    ros::Subscriber areoEstiImuAccNoiseCov_sub;
    ros::Subscriber areoEstiOpticalFlowVelAndDistNoiseCov_sub;
    ros::Subscriber areoEstiOpticalFlowQualInitAndThresh_sub;
    ros::Subscriber areoEstiSonarKfParam_sub;
    ros::Subscriber areoEstiSonarCompParam_sub;
    ros::Subscriber areoEstiMotionModelAccNoiseCov_sub;
    ros::Subscriber areoEstiMotionModelAngVelAndHeightNoiseCov_sub;

    void areodynamicRunModeCb(const std_msgs::Float32 &msg);
    void dragForceParamCb(const geometry_msgs::Vector3 &msg);
    void imuAccNoiseCovCb(const geometry_msgs::Vector3 &msg);
    void opticalFlowVelAndDistNoiseCovCb(const geometry_msgs::Vector3 &msg);
    void opticalFlowQualInitAndThreshCb(const geometry_msgs::Vector3 &msg);
    void sonarKfParamCb(const geometry_msgs::Vector3 &msg);
    void sonarCompParamCb(const geometry_msgs::Vector3 &msg);
    void motionModelAccNoiseCovCb(const geometry_msgs::Vector3 &msg);
    void MotionModelAngVelAndHeightNoiseCovCb(const geometry_msgs::Vector3 &msg);


//*****************************************************

//***************kf for filter magnet data**************
    double magKfP,magKfR,magKfQ,magKfK;
    double magKfX,magKfY,magKfZ;

//**********************************************************

//***************kf for estimate yaw************************
    double estiYawKfP,estiYawKfR,estiYawKfQ,estiYawKfK;
//**********************************************************

//***************kf for filter barometer data***********
    double baroKfP,baroKfR,baroKfQ,baroKfK;
    double heightFromBaroKfFilted;
//******************************************************

//**************complementary filter for barometer******
    double compK1,compK2,compK3;
    double heightFromBaroCompFilted,velFromBaroCompFilted,compBias;
//******************************************************

//***************kf for filter sonar data***********
    double sonarKfP,sonarKfR,sonarKfQ,sonarKfK;
    double heightFromSonarKfFilted;
//******************************************************

//**************complementary filter for sonar******
    double sonarCompK1,sonarCompK2,sonarCompK3;
    double heightFromSonarCompFilted,velFromSonarCompFilted,sonarCompBias;
//******************************************************

public:
    void startMotionEstiLoop();
    void MotionEstiLoop();
    void startSpinOnceLoop();
    void spinOnceLoop();

    void updateSensorsAndVicon();
    void calcuYawFromMag();
    void calculatePath(double dt);
    void magKfFilter();
    void estiYawKf(double dt);
    void baroKfFilter();
    void baroCompFilter(double dt);
    void calcuBodyVz();
    void sonarKfFilter();
    void sonarCompFilter(double dt);


private:
    ros::NodeHandle nh;
    Vicon *viconObj;
    UavSensors *uavSensorsObj;
    pthread_t read_tId,read_tIdSpin;
    uav_math *uavMathObj;
    bool setup();

};
#endif // UAV_MOTION_ESTIMATION

