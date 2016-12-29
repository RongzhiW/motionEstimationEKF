
/*mediaSocUAV
 * 先验模型用科里奥地力，测量模型不用科里奥地力;先验模型中z方向的本地
  加速度等式中z方向加速度计值和科里奥地利无关
  磁力计，气压计融合加速度计、陀螺仪获得方位角和高度
  don't estimate the dragForce Param
  fuse optical_flow,sonar
  calibrate the imu delay problem
  统一了代码风格
  移植到平台上
  添加了用于调试的一些topic
  去除了姿态的估计，目前状态只有角速率的bias，XY速度，高度
*/
#include "uav_motion_estimation_ekf.h"
#include <fstream>


using namespace std;
//const char dir_path[] = "/home/tianu/Record";
ofstream f1("/home/tianu/Record/motionEstimationData/mediaSocUAV/data.txt", ios::out);
ofstream f2("/home/tianu/Record/motionEstimationData/mediaSocUAV/test.txt",ios::out);
ofstream f3("/home/tianu/Record/motionEstimationData/mediaSocUAV/forCalib.txt",ios::out);

bool UavMotionEstimationEKF::setup(){
    if(!f1){
        cout << "open file failed!" << endl;
        return 0;
    }
    //because there is some dift between imu and vicon|optical_flow
    arraySize=20;
    drift=8;
    OpticalFlowDatasArray=new OpticalFlowData*[arraySize];
    for(int i=0;i<arraySize;++i)
        OpticalFlowDatasArray[i]=new OpticalFlowData;
    viconPointArray=new ViconPoint*[arraySize];
    for(int i=0;i<arraySize;++i)
        viconPointArray[i]=new ViconPoint;

    isFirstWrite=true;
    coutOnceFlag=true;
    uavMathObj=new uav_math();

    //ekf initialize
//    measFlowVxNoiseCovInit=0.0253;
//    measFlowVyNoiseCovInit=0.0253;//for bright_day in uav room
//    measFlowVxNoiseCovInit=0.00153;
//    measFlowVyNoiseCovInit=0.00153;//for bright_night in uav room 1
    measFlowVxNoiseCovInit=0.00553;
    measFlowVyNoiseCovInit=0.00553;//for bright_night in uav room 2
    measFlowDistNoiseCovInit=0.00275;
    measImuAxtNoiseCov=0.01;
    measImuAytNoiseCov=0.0107; // for bag 201612131519 pixhawk

//    measFlowVxNoiseCovInit=0.0003753;
//    measFlowVyNoiseCovInit=0.0003753;
//    measFlowDistNoiseCovInit=0.0275;
//    measImuAxtNoiseCov=0.000019;
//    measImuAytNoiseCov=0.0000207;//for pixhawkImu

    flowQualInitial=250.0;
    measFlowNoiseCovUpdateInterval=0;
    qualLowThresh=50;
    qualHighThresh=254;
    velLowThresh=0.05;
    averageQual=0;
    sumQual=0;
    processCounter=0;
    x=VectorXd::Zero(n_state);
//    dragForceParamX = -0.2001;
//    dragForceParamY = -0.3188; //bag in 201610252132—mpu9250
//    dragForceParamX = -0.2436;
//    dragForceParamY = -0.2825; //bag in 201610252132—pixhawk
    dragForceParamX = -0.2748;
    dragForceParamY = -0.2498; //bag in 201612131524—pixhawk
//    dragForceParamX = -0.2799;
//    dragForceParamY = -0.2031; //bag in 201612131519—pixhawk

    P=MatrixXd::Identity(n_state,n_state);
    Q=MatrixXd::Zero(8,8);
    Q(0,0)=0.0031;Q(1,1)=0.0031;
    Q(2,2)=0.000006042;Q(3,3)=0.000006007;
    Q(4,4)=0.5846;
    Q(5,5)=0.27;
    Q(6,6)=0.3319;
    Q(7,7)=0.00275; //bag 201611162213
//    Q(4,4)=0.000003849;
//    Q(5,5)=0.0000007;
//    Q(6,6)=0.000001319; //bag 201611162213 for pixhawk

    measurement_R=MatrixXd::Zero(5,5);
    measurement_R(0,0)=measImuAxtNoiseCov;
    measurement_R(1,1)=measImuAytNoiseCov;
    measurement_R(2,2) = measFlowVxNoiseCovInit;
    measurement_R(3,3) = measFlowVyNoiseCovInit;
    measurement_R(4,4) = measFlowDistNoiseCovInit;

    imuEmpty=true;
    initialized=false;
    testFlag=false;
    imuDataStamp = 0;
    imuDataStampLast = 0;

    //use imu_raw and sonar_raw by default
    useImuRaw=true;useImuRollAndPitchKfFused=false;useImuYawfusedWithMag=false;
    useSonarRaw=true;useSonarKf=false;useSonarComp=false;

    gyroBiasTuo=100;

//    estiBodyVel_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/body_vel",10);
//    estiRPY_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/euler_angle",10);
////    estiWorldVel_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/world_vel",10);
////    estiWorldVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/est_vel",10);
//    estiWorldVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/est_vel",10);
//    heightFromSonar_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/height",10);
////    estiWorldPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/areodynamic_optical_flow/pose",10);
//    estiWorldPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/est_pose",10);

//    areoEstiRunMode_sub = nh.subscribe("/areodynamic_optical_flow/areodynamicRunMode", 1, &UavMotionEstimationEKF::areodynamicRunModeCb,this);
//    areoEstiDragForceParam_sub = nh.subscribe("/areodynamic_optical_flow/dragForceParam", 1, &UavMotionEstimationEKF::dragForceParamCb,this);
//    areoEstiImuAccNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/imuAccNoiseCov", 1, &UavMotionEstimationEKF::imuAccNoiseCovCb,this);
//    areoEstiOpticalFlowVelAndDistNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/OpticalFlowVelAndDistNoiseCov", 1, &UavMotionEstimationEKF::opticalFlowVelAndDistNoiseCovCb,this);
//    areoEstiMotionModelAccNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/motionModelAccNoise", 1, &UavMotionEstimationEKF::motionModelAccNoiseCovCb,this);
//    areoEstiMotionModelAngVelAndHeightNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/MotionModelAngVelAndHeightNoise", 1, &UavMotionEstimationEKF::MotionModelAngVelAndHeightNoiseCovCb,this);

//    areoEstiOpticalFlowQualInitAndThresh_sub = nh.subscribe("/areodynamic_optical_flow/opticalFlowQualInitAndThresh", 1, &UavMotionEstimationEKF::opticalFlowQualInitAndThreshCb,this);
//    areoEstiSonarKfParam_sub = nh.subscribe("/areodynamic_optical_flow/sonarKfParam", 1, &UavMotionEstimationEKF::sonarKfParamCb,this);
//    areoEstiSonarCompParam_sub = nh.subscribe("/areodynamic_optical_flow/sonarCompParam", 1, &UavMotionEstimationEKF::sonarCompParamCb,this);

//    //***************for debug*******************
//    opticalFlowPosePub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowPoseDebug",1);
//    opticalFlowBodyVelPub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowBodyVelDebug",1);
//    opticalFlowWorldVelPub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowWorldVelDebug",1);
//    viconPosePub = nh.advertise<geometry_msgs::Vector3>("/viconPoseDebug",1);
//    viconBodyVelPub = nh.advertise<geometry_msgs::Vector3>("/viconBodyVelDebug",1);
//    viconWorldVelPub = nh.advertise<geometry_msgs::Vector3>("/viconWolrdVelDebug",1);
//    viconRPYPub = nh.advertise<geometry_msgs::Vector3>("/viconRPYDebug",1);
//    imuAccRawPub  = nh.advertise<geometry_msgs::Vector3>("/imuAccRawDebug",1);
//    imuAccPub  = nh.advertise<geometry_msgs::Vector3>("/imuAccDebug",1);
//    imuRPYRawPub  = nh.advertise<geometry_msgs::Vector3>("/imuRPYRawDebug",1);
//    //*******************************************

    estiBodyVel_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/body_vel_1",10);
    estiRPY_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/euler_angle_1",10);
//    estiWorldVel_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/world_vel",10);
//    estiWorldVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/est_vel",10);
    estiWorldVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/est_vel_1",10);
    heightFromSonar_pub = nh.advertise<geometry_msgs::Vector3>("/areodynamic_optical_flow/height_1",10);
//    estiWorldPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/areodynamic_optical_flow/pose",10);
    estiWorldPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/est_pose_1",10);

    areoEstiRunMode_sub = nh.subscribe("/areodynamic_optical_flow/areodynamicRunMode", 1, &UavMotionEstimationEKF::areodynamicRunModeCb,this);
    areoEstiDragForceParam_sub = nh.subscribe("/areodynamic_optical_flow/dragForceParam", 1, &UavMotionEstimationEKF::dragForceParamCb,this);
    areoEstiImuAccNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/imuAccNoiseCov", 1, &UavMotionEstimationEKF::imuAccNoiseCovCb,this);
    areoEstiOpticalFlowVelAndDistNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/OpticalFlowVelAndDistNoiseCov", 1, &UavMotionEstimationEKF::opticalFlowVelAndDistNoiseCovCb,this);
    areoEstiMotionModelAccNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/motionModelAccNoise", 1, &UavMotionEstimationEKF::motionModelAccNoiseCovCb,this);
    areoEstiMotionModelAngVelAndHeightNoiseCov_sub = nh.subscribe("/areodynamic_optical_flow/MotionModelAngVelAndHeightNoise", 1, &UavMotionEstimationEKF::MotionModelAngVelAndHeightNoiseCovCb,this);

    areoEstiOpticalFlowQualInitAndThresh_sub = nh.subscribe("/areodynamic_optical_flow/opticalFlowQualInitAndThresh", 1, &UavMotionEstimationEKF::opticalFlowQualInitAndThreshCb,this);
    areoEstiSonarKfParam_sub = nh.subscribe("/areodynamic_optical_flow/sonarKfParam", 1, &UavMotionEstimationEKF::sonarKfParamCb,this);
    areoEstiSonarCompParam_sub = nh.subscribe("/areodynamic_optical_flow/sonarCompParam", 1, &UavMotionEstimationEKF::sonarCompParamCb,this);

    //***************for debug*******************
    opticalFlowPosePub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowPoseDebug_1",1);
    opticalFlowBodyVelPub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowBodyVelDebug_1",1);
    opticalFlowWorldVelPub = nh.advertise<geometry_msgs::Vector3>("/opticalFlowWorldVelDebug_1",1);
    viconPosePub = nh.advertise<geometry_msgs::Vector3>("/viconPoseDebug_1",1);
    viconBodyVelPub = nh.advertise<geometry_msgs::Vector3>("/viconBodyVelDebug_1",1);
    viconWorldVelPub = nh.advertise<geometry_msgs::Vector3>("/viconWolrdVelDebug_1",1);
    viconRPYPub = nh.advertise<geometry_msgs::Vector3>("/viconRPYDebug_1",1);
    imuAccRawPub  = nh.advertise<geometry_msgs::Vector3>("/imuAccRawDebug_1",1);
    imuAccPub  = nh.advertise<geometry_msgs::Vector3>("/imuAccDebug_1",1);
    imuRPYRawPub  = nh.advertise<geometry_msgs::Vector3>("/imuRPYRawDebug_1",1);
    //*******************************************
    worldPoseX=0;
    worldPoseY=0;
    worldPoseZ=0;

    isFirstCalcuYaw=true;
    isFirstUpdateData=true;

    //**********initialize for magnet kf***********
    magKfP=1.0;magKfQ=0.00001;magKfR=0.01;magKfK=0;
    magKfX=0;magKfY=0;magKfZ=0;

    //*********initialize for estimate yaw kf****
    estiYawKfP=1.0;estiYawKfQ=0.00001;estiYawKfR=0.01;estiYawKfK=0;
    estiYawValue2PI=0;

    //*********initialize for baro kf************
    baroKfP=1.0;baroKfQ=0.001;baroKfR=0.1;baroKfK=0;
    heightFromBaroKfFilted = 0;

    //*********iintialize for baro complementary filter*******
    compK1=0.45;compK2=0.45;compK3=0.0001;
    heightFromBaroCompFilted=0;
    velFromBaroCompFilted=0;
    compBias=0;

    //*********initialize for sonar kf************
    sonarKfP=1.0;sonarKfQ=0.01;sonarKfR=0.125;sonarKfK=0;
    heightFromSonarKfFilted = 0;

    //*********iintialize for sonar complementary filter*******
    sonarCompK1=3.57;sonarCompK2=6.14;sonarCompK3=0.0001;
    heightFromSonarCompFilted=0;
    velFromSonarCompFilted=0;
    sonarCompBias=0;
    return 1;
}

void UavMotionEstimationEKF::startMotionEstiLoop(){
    pthread_create( &read_tId, NULL, &startMotionEstiThread, this );
}

void UavMotionEstimationEKF::startSpinOnceLoop(){
    pthread_create(&read_tIdSpin,NULL,&startSpinOnceThread,this);
}

void *startMotionEstiThread(void *args){
    UavMotionEstimationEKF *w=(UavMotionEstimationEKF*)args;
    w->MotionEstiLoop();
    return NULL;
}

void *startSpinOnceThread(void *args){
    UavMotionEstimationEKF *w=(UavMotionEstimationEKF*)args;
    w->spinOnceLoop();
    return NULL;
}

void UavMotionEstimationEKF::spinOnceLoop(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        //cout<<"spinOnce"<<endl;
        loop_rate.sleep();
    }
}
void UavMotionEstimationEKF::MotionEstiLoop(){
    ros::Rate loop_rate(20);
    while (ros::ok() && !threadQuit){

        if(uavSensorsObj->isImuAvaible){
//        if(uavSensorsObj->isImuAvaible && viconObj->isViconAvaible){
            updateSensorsAndVicon();
            imuDataStamp = imuData.timeStamp;

            if(abs(imuDataStamp-imuDataStampLast)>0.0001){
                //************start EKF***************
                double timeNow;
                timeNow = ros::Time::now().toSec();
                processSensorData(timeNow);
                //************************************

                //log data
                if(isFirstWrite){
                    f1<<"estiRoll "<<"estiPitch "<<"estiBodyVx "<<"estiBodyVy "<<"estiBodyVz "<<"viconRoll "<<"viconPitch "<<"viconYaw "<<"viconBodyVx "\
                                         <<"viconBodyVy "<<"viconBodyVz "<<"viconPoseX "<<"viconPoseY "<<"viconPoseZ "<<"imuRoll "<<"imuPitch "<<"imuYaw "\
                                        <<"bodyVzFromSonar "<<"estiYawValue "<<"opticalFlowWx "<<\
                                          "opticalFlowWy "<<"opticalFlowBvx "<<"opticalFlowBvy "<<"opticalFlowBvz "<<"imuAngX "<<"imuAngY "<<"imuAngZ "<<"imuAx "\
                                        <<"imuAy "<<"imuAz "<<"viconBodyAx "<<"viconBodyAy "<<"viconBodyAz "<<"heightFromSonarKfFilted "\
                                       <<"viconWorldVx "<<"viconWorldVy "<<"viconWorldVz "<<"opticalFlowWvx "<<"opticalFlowWvy "<<"imuAxRaw "<<"imuAyRaw "<<"imuAzRaw "\
                                      <<"opticalFlowDis "<<"estiHeight "<<"estiWorldVx "<<"estiWorldVy "<<"estiWorldVz "<<"opticalFlowDisRaw"<<endl;
                    f2<<"Ax "<<"Ay "<<"Az "<<"Wx "<<"Wy "<<"Wz"<<endl;
                    f3<<"viconRoll "<<"viconPitch "<<"viconYaw "<<"viconBodyVx "<<"viconBodyVy "<<"viconBodyVz "<<"viconBodyAx "<<"viconBodyAy "<<"viconBodyAz "\
                     <<"imuAngX "<<"imuAngY "<<"imuAngZ"<<endl;
                    isFirstWrite=false;
                }

                else{
                    f1<<imuData.roll<<' '<<imuData.pitch<<' '<<x(2)<<' '<<x(3)<<' '<<x(4)<<' '<<viconData.roll<<' '<<viconData.pitch<<' '<<viconData.yaw<<' '<<viconData.vx<<\
                                            ' '<<viconData.vy<<' '<<viconData.vz<<' '<<viconData.x<<' '<<viconData.y<<' '<<viconData.z<<' '<<imuData.roll<<' '<<imuData.pitch\
                                         <<' '<<imuData.yaw<<' '<<bodyVzFromSonar<<' '<<estiYawValue<<' '\
                                        <<opticalFlowData.opticalGlobal_x_m<<' '<<opticalFlowData.opticalGlobal_y_m<<\
                                          ' '<<opticalFlowData.opticalLocal_vx<<' '<<opticalFlowData.opticalLocal_vy<<' '<<opticalFlowData.opticalLocal_vz\
                                        <<' '<<imuData.angularVx<<' '<<imuData.angularVy<<' '<<imuData.angularVz<<' '<<imuData.accelera_x<<' '<<imuData.accelera_y<<' '\
                                        <<imuData.accelera_z<<' '<<viconData.ax<<' '<<viconData.ay<<' '<<viconData.az<<' '<<heightFromSonarKfFilted\
                                       <<' '<<viconData.worldVx<<' '<<viconData.worldVy<<' '<<viconData.worldVz<<' '<<opticalFlowData.opticalGlobal_vx<<' '\
                                      <<opticalFlowData.opticalGlobal_vy<<' '<<imuData.acceleraRaw_x<<' '<<imuData.acceleraRaw_y<<' '<<imuData.acceleraRaw_z<<' '\
                                     <<opticalFlowData.distance<<' '<<x(5)<<' '<<worldVel[0]<<' '<<worldVel[1]<<' '<<worldVel[2]<<' '<<opticalFlowData.distanceRaw<<endl;

                    f2<<imuAcc(0)<<' '<<imuAcc(1)<<' '<<imuAcc(2)<<' '<<bodyAngularVel(0)<<' '<<bodyAngularVel(1)<<' '<<bodyAngularVel(2)<<endl;
                    f3<<viconData.roll<<' '<<viconData.pitch<<' '<<viconData.yaw<<' '<<viconData.vx<<' '<<viconData.vy<<' '<<viconData.vz<<' '\
                     <<viconData.ax<<' '<<viconData.ay<<' '<<viconData.az<<' '<<imuData.angularVx<<' '<<imuData.angularVy<<' '<<imuData.angularVz<<endl;
                }

                //cout<<"estiBodyVx:"<<x(2)<<" viconBodyVx:"<<viconData.vx<<" estiBodyVy:"<<x(3)<<" viconBodyVy:"<<viconData.vy<<" estiBodyVz:"<<x(4)\
                   <<" viconBodyVz:"<<viconData.vz<<" gyroxBias:"<<x(0)<<" gyroyBias:"<<x(1)<<endl;
            }
            imuDataStampLast = imuDataStamp;

        }
        else{
            cout<<"imu stops updating"<<endl;
        }

        loop_rate.sleep();
    }
}

void UavMotionEstimationEKF::areodynamicRunModeCb(const std_msgs::Float32 &msg){
    if(msg.data == 1){
        useImuRaw=true;useImuRollAndPitchKfFused=false;useImuYawfusedWithMag=false;
    }else if(msg.data == 2){
        useImuRaw=false;useImuRollAndPitchKfFused=true;useImuYawfusedWithMag=false;
    }else if(msg.data == 3){
        useImuRaw=false;useImuRollAndPitchKfFused=true;useImuYawfusedWithMag=true;
    }else if(msg.data == 4){
        useSonarRaw=true;useSonarKf=false;useSonarComp=false;
    }else if(msg.data == 5){
        useSonarRaw=false;useSonarKf=true;useSonarComp=false;
    }else if(msg.data == 6){
        useSonarRaw=false;useSonarKf=false;useSonarComp=true;
    }else{
        ROS_INFO("wrong run mode!");
    }
    cout<<"mode:"<<msg.data<<endl;

}

void UavMotionEstimationEKF::dragForceParamCb(const geometry_msgs::Vector3 &msg){
       if(msg.x<0 && msg.x>-1){
           dragForceParamX=msg.x;
       }
       if(msg.y<0 && msg.y>-1){
           dragForceParamY=msg.y;
       }
       cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::imuAccNoiseCovCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        measurement_R(0,0)=msg.x;
    }
    if(msg.y>0){
        measurement_R(1,1)=msg.y;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::opticalFlowVelAndDistNoiseCovCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        measFlowVxNoiseCovInit=msg.x;
    }
    if(msg.y>0){
        measFlowVyNoiseCovInit=msg.y;
    }
    if(msg.z>0){
        measFlowDistNoiseCovInit = msg.z;
        measurement_R(4,4) = msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::motionModelAccNoiseCovCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        Q(4,4)=msg.x;
    }
    if(msg.y>0){
        Q(5,5)=msg.y;
    }
    if(msg.z>0){
        Q(6,6) = msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::opticalFlowQualInitAndThreshCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0 && msg.x<=255){
        flowQualInitial=msg.x;
    }
    if(msg.y>0 && msg.y<=255){
        qualLowThresh=msg.y;
    }
    if(msg.z>0 && msg.z<=255){
        qualHighThresh=msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::sonarKfParamCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        sonarKfP=msg.x;
    }
    if(msg.y>0){
        sonarKfQ=msg.y;
    }
    if(msg.z>0){
        sonarKfR=msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::sonarCompParamCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        sonarCompK1=msg.x;
    }
    if(msg.y>0){
        sonarCompK2=msg.y;
    }
    if(msg.z>0){
        sonarCompK3=msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;

}

void UavMotionEstimationEKF::MotionModelAngVelAndHeightNoiseCovCb(const geometry_msgs::Vector3 &msg){
    if(msg.x>0){
        Q(0,0)=msg.x;
    }
    if(msg.y>0){
        Q(1,1)=msg.y;
    }
    if(msg.z>0){
        Q(7,7) = msg.z;
    }
    cout<<"x:"<<msg.x<<" y:"<<msg.y<<" z:"<<msg.z<<endl;
}

void UavMotionEstimationEKF::updateSensorsAndVicon(){
    //update imu
    imuData.accelera_x = uavSensorsObj->imuData.accelera_x;
    imuData.accelera_y = uavSensorsObj->imuData.accelera_y;
    imuData.accelera_z = uavSensorsObj->imuData.accelera_z;
    imuData.acceleraRaw_x = uavSensorsObj->imuData.acceleraRaw_x;
    imuData.acceleraRaw_y = uavSensorsObj->imuData.acceleraRaw_y;
    imuData.acceleraRaw_z = uavSensorsObj->imuData.acceleraRaw_z;
    imuData.pitch = uavSensorsObj->imuData.pitch;
    imuData.roll = uavSensorsObj->imuData.roll;
    imuData.yaw = uavSensorsObj->imuData.yaw;
    imuData.x = uavSensorsObj->imuData.x;
    imuData.y = uavSensorsObj->imuData.y;
    imuData.z = uavSensorsObj->imuData.z;
    imuData.w = uavSensorsObj->imuData.w;
    imuData.angularVx = uavSensorsObj->imuData.angularVx;
    imuData.angularVy = uavSensorsObj->imuData.angularVy;
    imuData.angularVz = uavSensorsObj->imuData.angularVz;
    imuData.timeStamp = uavSensorsObj->imuData.timeStamp;

    //************************for debug***********************
    //transform to NWU for mrh's control
     //transform to NEU for mrh's control
    geometry_msgs::Vector3 imuAccRawDebug,imuAccDebug,imuRPYdebug;
    imuAccRawDebug.x = imuData.acceleraRaw_x;
    imuAccRawDebug.y = imuData.acceleraRaw_y;
    imuAccRawDebug.z = -imuData.acceleraRaw_z;
    //there is a constant bias between imu and vicon
    double axBias=-0.1820;
    double ayBias=0;
    imuAccDebug.x = (imuData.accelera_x-axBias)/dragForceParamX;
    imuAccDebug.y = (imuData.accelera_y-ayBias)/dragForceParamY;
    imuAccDebug.z = -imuData.accelera_z;
    //calculate the total horizonal acceleration
//    imuAccDebug.z = sqrt(imuData.accelera_x*imuData.accelera_x+imuData.accelera_y*imuData.accelera_y)/0.25;
    imuRPYdebug.x = imuData.roll;
    imuRPYdebug.y = imuData.pitch;
    imuRPYdebug.z = -imuData.yaw;
    imuAccRawPub.publish(imuAccRawDebug);
    imuAccPub.publish(imuAccDebug);
    imuRPYRawPub.publish(imuRPYdebug);
    //***********************************************************

    //update optical_flow
    opticalFlowData.distance = uavSensorsObj->opticalFlowData.distance;
    opticalFlowData.distanceRaw = uavSensorsObj->opticalFlowData.distanceRaw;
    opticalFlowData.opticalGlobal_vx = uavSensorsObj->opticalFlowData.opticalGlobal_vx;
    opticalFlowData.opticalGlobal_vy = uavSensorsObj->opticalFlowData.opticalGlobal_vy;
    opticalFlowData.opticalGlobal_vz = uavSensorsObj->opticalFlowData.opticalGlobal_vz;
    opticalFlowData.opticalLocal_vx = uavSensorsObj->opticalFlowData.opticalLocal_vx;
    opticalFlowData.opticalLocal_vy = uavSensorsObj->opticalFlowData.opticalLocal_vy;
    opticalFlowData.opticalLocal_vz = uavSensorsObj->opticalFlowData.opticalLocal_vz;
    opticalFlowData.opticalGlobal_x_m = uavSensorsObj->opticalFlowData.opticalGlobal_x_m;
    opticalFlowData.opticalGlobal_y_m = uavSensorsObj->opticalFlowData.opticalGlobal_y_m;
    opticalFlowData.quality = (uavSensorsObj->opticalFlowData.quality==0)?1:uavSensorsObj->opticalFlowData.quality;
    opticalFlowData.timeStamp = uavSensorsObj->opticalFlowData.timeStamp;

//    for(int i=arraySize-2;i>=0;--i){
//            *(OpticalFlowDatasArray[i+1])=*(OpticalFlowDatasArray[i]);
//    }
    //*(OpticalFlowDatasArray[0])=opticalFlowData;

    //************************for debug***********************
    //transform to NEU for mrh's control
    geometry_msgs::Vector3 opticalFlowPoseDebug,opticalFlowBodyVelDebug,opticalFlowWorldVelDebug;
    opticalFlowPoseDebug.x = opticalFlowData.opticalGlobal_x_m;
    opticalFlowPoseDebug.y = opticalFlowData.opticalGlobal_y_m;
    opticalFlowPoseDebug.z = -opticalFlowData.distanceRaw;
    opticalFlowBodyVelDebug.x = opticalFlowData.opticalLocal_vx;
    opticalFlowBodyVelDebug.y = opticalFlowData.opticalLocal_vy;
    opticalFlowBodyVelDebug.z = -opticalFlowData.opticalLocal_vz;
    //calculate the total horizonal optical velocity
//    double flowVxDebug=opticalFlowData.opticalLocal_vx;
//    double flowVyDebug=opticalFlowData.opticalLocal_vy;
//    opticalFlowBodyVelDebug.z = sqrt(flowVxDebug*flowVxDebug+flowVyDebug*flowVyDebug);

    opticalFlowWorldVelDebug.x = opticalFlowData.opticalGlobal_vx;
    opticalFlowWorldVelDebug.y = opticalFlowData.opticalGlobal_vy;
    opticalFlowWorldVelDebug.z = -opticalFlowData.opticalGlobal_vz;
    opticalFlowPosePub.publish(opticalFlowPoseDebug);
    opticalFlowBodyVelPub.publish(opticalFlowBodyVelDebug);
    opticalFlowWorldVelPub.publish(opticalFlowWorldVelDebug);
    //***********************************************************

    // update vicon, vicon pose is NWU, need to transform from NWU to NED
    viconData.ax = viconObj->state.ax;
    viconData.ay = -viconObj->state.ay;
    viconData.az = -viconObj->state.az;
    viconData.vx = viconObj->state.vx;
    viconData.vy = -viconObj->state.vy;
    viconData.vz = -viconObj->state.vz;
    viconData.bodyAngX = viconObj->state.bodyAngX;
    viconData.bodyAngY = -viconObj->state.bodyAngY;
    viconData.bodyAngZ = -viconObj->state.bodyAngZ;

    viconData.x = viconObj->state.x;
    viconData.y = -viconObj->state.y;
    viconData.z = -viconObj->state.z;
    viconData.pitch = -viconObj->state.pitch;
    viconData.roll = viconObj->state.roll;
    viconData.yaw = -viconObj->state.yaw;
    viconData.worldVx = viconObj->state.worldVx;
    viconData.worldVy = -viconObj->state.worldVy;
    viconData.worldVz = -viconObj->state.worldVz;
    viconData.timeStamp = viconObj->state.timeStamp;

//    for(int i=arraySize-2;i>=0;--i){
//            *(viconPointArray[i+1])=*(viconPointArray[i]);
//    }
//    *(viconPointArray[0])=viconData;

    //**************for debug*********************
    //transform to NEU for mrh's control
    geometry_msgs::Vector3 viconPoseDebug;
    geometry_msgs::Vector3 viconBodyVelDebug,viconRPYDebug,viconWorldVelDebug;
    viconPoseDebug.x = viconData.x;
    viconPoseDebug.y = viconData.y;
    viconPoseDebug.z = -viconData.z;
    viconBodyVelDebug.x= viconData.vx;
    viconBodyVelDebug.y= viconData.vy;
    viconBodyVelDebug.z= -viconData.vz;
    //calculate the total horizonal vicon velocity
//    double viconVxDebug=viconData.vx;
//    double viconVyDebug=viconData.vy;
//    viconBodyVelDebug.z = sqrt(viconVxDebug*viconVxDebug+viconVyDebug*viconVyDebug);

    viconWorldVelDebug.x = viconData.worldVx;
    viconWorldVelDebug.y = viconData.worldVy;
    viconWorldVelDebug.z = -viconData.worldVz;
    viconRPYDebug.x = viconData.roll;
    viconRPYDebug.y = viconData.pitch;
    viconRPYDebug.z = -viconData.yaw;
    viconPosePub.publish(viconPoseDebug);
    viconBodyVelPub.publish(viconBodyVelDebug);
    viconWorldVelPub.publish(viconWorldVelDebug);
    viconRPYPub.publish(viconRPYDebug);
    //********************************************

    //update barometer and magnet,trans to NED
    heightFromBaro = -uavSensorsObj->uav_01HeightFromBaro;
    uav_01Mag.x = uavSensorsObj->uav_01Mag.x;
    uav_01Mag.y = uavSensorsObj->uav_01Mag.y;
    uav_01Mag.z = uavSensorsObj->uav_01Mag.z;

    //cout<<"bodyVel "<<"body_vx:"<<bodyVel[0]<<" body_vy:"<<bodyVel[1]<<" body_vz:"<<bodyVel[2]<<endl;

}

void UavMotionEstimationEKF::predict(Vector3d gyro, Vector3d imuAcc,double t){

    if(t-start_t<=current_t)return;
    double dt=t-start_t-current_t;
    VectorXd xdot(n_state);
    MatrixXd A(n_state,n_state);
    MatrixXd W(n_state,8);

    process(gyro,imuAcc,xdot,A,W);

    x += xdot*dt;

    A=MatrixXd::Identity(n_state,n_state)+A*dt;
    W=W*dt;

    P=A*P*A.transpose()+W*Q*W.transpose();
}

void UavMotionEstimationEKF::process(Vector3d gyro,Vector3d imuAcc,VectorXd& xdot, MatrixXd& A, MatrixXd& W){
    Vector2d rp,gyroBias;
    Vector3d bodyVelTmp;
    double height;
    getState(rp,gyroBias,bodyVelTmp,height);

    xdot.setZero();
    A.setZero();
    W.setZero();

    xdot(0)=-1/gyroBiasTuo*gyroBias(0);
    xdot(1)=-1/gyroBiasTuo*gyroBias(1);

    xdot(2)=-GRAVITY(2)*sin(rp(1))+dragForceParamX*bodyVelTmp(0)+gyro(2)*bodyVelTmp(1)-(gyro(1)-gyroBias(1))*bodyVelTmp(2);
    xdot(3)=GRAVITY(2)*cos(rp(1))*sin(rp(0))+dragForceParamY*bodyVelTmp(1)+(gyro(0)-gyroBias(0))*bodyVelTmp(2)-gyro(2)*bodyVelTmp(0);
    xdot(4)=GRAVITY(2)*cos(rp(0))*cos(rp(1))+imuAcc(2)+(gyro(1)-gyroBias(1))*bodyVelTmp(0)-(gyro(0)-gyroBias(0))*bodyVelTmp(1);

//    xdot(7)=-sin(viconData.pitch)*bodyVelTmp(0)+cos(viconData.pitch)*sin(viconData.roll)*bodyVelTmp(1)+cos(viconData.pitch)*cos(viconData.roll)*bodyVelTmp(2);
    xdot(5)=-sin(rp(1))*bodyVelTmp(0)+cos(rp(1))*sin(rp(0))*bodyVelTmp(1)+cos(rp(1))*cos(rp(0))*bodyVelTmp(2);


    A(0,0) = -1/gyroBiasTuo;

    A(1,1) = -1/gyroBiasTuo;

    A(2,1) = bodyVelTmp(2);
    A(2,2) = dragForceParamX;
    A(2,3) = gyro(2);
    A(2,4) = -(gyro(1)-gyroBias(1));

    A(3,0) = -bodyVelTmp(2);
    A(3,2) = -gyro(2);
    A(3,3) = dragForceParamY;
    A(3,4) = (gyro(0)-gyroBias(0));

    A(4,0) = bodyVelTmp(1);
    A(4,1) = -bodyVelTmp(0);
    A(4,2) = gyro(1)-gyroBias(1);
    A(4,3) = -(gyro(0)-gyroBias(0));

    A(5,2) = -sin(rp(1));
    A(5,3) = cos(rp(1))*sin(rp(0));
    A(5,4) = cos(rp(1))*cos(rp(0));

    W.block<4,4>(0,2)=Matrix4d::Identity();
    W(2,1) = -bodyVelTmp(2);
    W(3,0) = bodyVelTmp(2);

    W(4,0) = -bodyVelTmp(1);
    W(4,1) = bodyVelTmp(0);
    W(4,6) = 1;

    W(5,7) = 1;
}

void UavMotionEstimationEKF::getState(Vector2d& rp, Vector2d& gyroBias,Vector3d& bodyVelTmp,double& height){
    rp(0)=imuData.roll;
    rp(1)=imuData.pitch;
    gyroBias=x.segment<2>(0);
    bodyVelTmp=x.segment<3>(2);
    height=x(5);
}

void UavMotionEstimationEKF::measurement(VectorXd& zhat, Vector3d gyro, MatrixXd &H, MatrixXd &V){
//        zhat(0)=dragForceParam1*x(4)+gyro(2)*x(5)-(gyro(1)-x(3))*x(6);
//        zhat(1)=dragForceParam1*x(5)+(gyro(0)-x(2))*x(6)-gyro(2)*x(4);
        zhat=VectorXd::Zero(5);
        zhat(0)=dragForceParamX*x(2);
        zhat(1)=dragForceParamY*x(3);
        zhat(2)=x(2);
        zhat(3)=x(3);
        zhat(4)=x(5);

        H=MatrixXd::Zero(5,n_state);
        H(0,2)=dragForceParamX;
        H(1,3)=dragForceParamY;
        H(2,2)=1;
        H(3,3)=1;
        H(4,5)=1;

        V=MatrixXd::Identity(5,5);
}

void UavMotionEstimationEKF::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R, MatrixXd V)
{
    MatrixXd K = P*H.transpose()*(H*P*H.transpose() + V*R*V.transpose()).inverse();
    x += K*(z - zhat);
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
}

void UavMotionEstimationEKF::correct_measurement(double t){
    if(t-start_t<current_t)return;
    predict(this->bodyAngularVel,this->imuAcc,t);
    double dt=t-start_t-current_t;
    //cout<<"dt:"<<dt<<endl;
    VectorXd z=VectorXd::Zero(5);
    //there is a constant bias between imu and vicon
    double axBias=-0.1820;
    double ayBias=0;
    z(0)=this->imuAcc(0)-axBias;
    z(1)=this->imuAcc(1)-ayBias;
//    z(2)=OpticalFlowDatasArray[drift]->opticalLocal_vx;
//    z(3)=OpticalFlowDatasArray[drift]->opticalLocal_vy;
    z(2)=opticalFlowData.opticalLocal_vx;
    z(3)=opticalFlowData.opticalLocal_vy;
    if(useSonarRaw){
        z(4)=-opticalFlowData.distance;
//        cout<<"measure mode 4"<<endl;
    }else if(useSonarKf){
        z(4)=heightFromSonarKfFilted;
//        cout<<"measure mode 5"<<endl;
    }else if(useSonarComp){
        z(4)=heightFromSonarCompFilted;
//        cout<<"measure mode 6"<<endl;
    }else{
        z(4)=-opticalFlowData.distance;
//        cout<<"measure mode 4"<<endl;
    }

    VectorXd zhat;
    MatrixXd H;
    MatrixXd V;
    measurement(zhat,this->bodyAngularVel,H,V);
    correct(z,zhat,H,measurement_R,V);
    //cout<<"current time:"<<current_t<<endl;
}

void UavMotionEstimationEKF::magKfFilter(){
    magKfX = magKfX; magKfY = magKfY; magKfZ = magKfZ;
    magKfP = magKfP+magKfQ;
    magKfK = magKfP/(magKfP+magKfR);
    double magKfZkX = uav_01Mag.x;
    double magKfZkY = uav_01Mag.y;
    double magKfZkZ = uav_01Mag.z;

    magKfX = magKfX+magKfK*(magKfZkX-magKfX);
    magKfY = magKfY+magKfK*(magKfZkY-magKfY);
    magKfZ = magKfZ+magKfK*(magKfZkZ-magKfZ);

    magKfP = (1-magKfR)*magKfP;
}

void UavMotionEstimationEKF::calcuYawFromMag(){
    double bodyMagX = -magKfY;
    double bodyMagY = -magKfX;
    double bodyMagZ = -magKfZ;
    double roll = imuData.roll;
    double pitch = imuData.pitch;
    //double roll = x(0);
    //double pitch = x(1);

    //姿态补偿
    double magXh,magYh;
    magXh = bodyMagX*cos(pitch)+bodyMagY*sin(pitch)*sin(roll)+\
            bodyMagZ*sin(pitch)*cos(roll);
    magYh = -bodyMagY*cos(roll)+bodyMagZ*sin(roll);

    if(magXh>0 && magYh>0){
        calcuYawValue = atan(magYh/magXh);
    }
    else if(magXh>0 && magYh<0){
        calcuYawValue = -atan(-magYh/magXh);
    }
    else if(magXh<0 && magYh>0){
        calcuYawValue = M_PI-atan(magYh/-magXh);
    }
    else if(magXh<0 && magYh<0){
        calcuYawValue = -M_PI+atan(magYh/magXh);
    }

    //cout<<"calcuYawValue:"<<calcuYawValue<<" viconYaw:"<<viconData.yaw<<endl;
}

void UavMotionEstimationEKF::estiYawKf(double dt){
    double roll = imuData.roll;
    double pitch = imuData.pitch;
    estiYawValue2PI = estiYawValue2PI+(this->bodyAngularVel[1]*sin(roll)/cos(pitch)+\
            this->bodyAngularVel[2]*cos(roll)/cos(pitch))*dt;
    estiYawKfP = estiYawKfP+estiYawKfQ;
    estiYawKfK = estiYawKfP/(estiYawKfP+estiYawKfR);
    double estiYawKfZk = calcuYawValue;
    estiYawValue2PI = estiYawValue2PI+estiYawKfK*(estiYawKfZk-estiYawValue2PI);
    estiYawKfP = (1-estiYawKfK)*estiYawKfP;

    //trans to -PI~PI
    estiYawValue = estiYawValue2PI>M_PI ? -2*M_PI+estiYawValue2PI : estiYawValue2PI;

    //cout<<"estiYawValue:"<<estiYawValue<<" imuYaw:"<<imuData.yaw<<" viconYaw:"<<viconData.yaw<<endl;
}

void UavMotionEstimationEKF::baroKfFilter(){
    heightFromBaroKfFilted = heightFromBaroKfFilted;
    baroKfP = baroKfP+baroKfQ;
    baroKfK = baroKfP/(baroKfP+baroKfR);
    double baroKfZk = heightFromBaro;
    heightFromBaroKfFilted = heightFromBaroKfFilted+baroKfK*(baroKfZk-heightFromBaroKfFilted);
    baroKfP = (1-baroKfK)*baroKfP;
    //cout<<"heightFromBaro:"<<heightFromBaro<<" heightFromBaroKfFilted:"<<heightFromBaroKfFilted<<endl;

}

void UavMotionEstimationEKF::baroCompFilter(double dt){
    double roll = imuData.roll;
    double pitch = imuData.pitch;
    double yaw = imuData.yaw;
    double accAx = imuData.accelera_x;
    double accAy = imuData.accelera_y;
    double accAz = imuData.accelera_z;
    float R_NED[9];
    uavMathObj->eulertoRotation(roll,pitch,yaw,R_NED);
    double worldAz;
    worldAz = R_NED[6]*accAx+R_NED[7]*accAy+R_NED[8]*accAz;
    double heightMeas = heightFromBaro;
    double heightFromBaroCompFiltedLast = heightFromBaroCompFilted;
    heightFromBaroCompFilted +=(velFromBaroCompFilted-compK1*(heightFromBaroCompFilted-heightMeas))*dt;
    velFromBaroCompFilted += (worldAz+GRAVITY[2]-compBias-compK2*(heightFromBaroCompFiltedLast-heightMeas))*dt;
    compBias += compK3*(heightFromBaroCompFiltedLast-heightMeas)*dt;
    //cout<<" wordAz:"<<worldAz+9.5<<" heightFromBaro:"<<heightFromBaro<<" heightFromBaroComp:"<<heightFromBaroCompFilted\
       <<" velFromBaroComp:"<<velFromBaroCompFilted<<" compBias:"<<compBias<<endl;

}

void UavMotionEstimationEKF::sonarKfFilter(){
    heightFromSonarKfFilted = heightFromSonarKfFilted;
    sonarKfP = sonarKfP+sonarKfQ;
    sonarKfK = sonarKfP/(sonarKfP+sonarKfR);
    double sonarKfZk = -opticalFlowData.distance;
    heightFromSonarKfFilted = heightFromSonarKfFilted+sonarKfK*(sonarKfZk-heightFromSonarKfFilted);
    sonarKfP = (1-sonarKfK)*sonarKfP;
    //cout<<"heightFromBaro:"<<heightFromBaro<<" heightFromBaroKfFilted:"<<heightFromBaroKfFilted<<endl;

}

void UavMotionEstimationEKF::sonarCompFilter(double dt){
    double roll = imuData.roll;
    double pitch = imuData.pitch;
    double yaw = imuData.yaw;
    double accAx = imuData.accelera_x;
    double accAy = imuData.accelera_y;
    double accAz = imuData.accelera_z;
    float R_NED[9];
    uavMathObj->eulertoRotation(roll,pitch,yaw,R_NED);
    double worldAz;
    worldAz = R_NED[6]*accAx+R_NED[7]*accAy+R_NED[8]*accAz+GRAVITY[2];
//    double heightMeasBody = -opticalFlowData.distance;
//    double heightMeasBody = heightFromSonarKfFilted;
//    double heightMeas = R_NED[8]*heightMeasBody;
    double heightMeas = -opticalFlowData.distance;

    double heightFromSonarCompFiltedLast = heightFromSonarCompFilted;
    heightFromSonarCompFilted +=(velFromSonarCompFilted-sonarCompK1*(heightFromSonarCompFilted-heightMeas))*dt;
    velFromSonarCompFilted += (worldAz-sonarCompBias-sonarCompK2*(heightFromSonarCompFiltedLast-heightMeas))*dt;
    sonarCompBias += sonarCompK3*(heightFromSonarCompFiltedLast-heightMeas)*dt;
    //cout<<" wordAz:"<<worldAz+9.5<<" heightFromBaro:"<<heightFromBaro<<" heightFromBaroComp:"<<heightFromBaroCompFilted\
       <<" velFromBaroComp:"<<velFromBaroCompFilted<<" compBias:"<<compBias<<endl;
}

void UavMotionEstimationEKF::calcuBodyVz(){
    double rollTmp,pitchTmp,yawTmp;
    rollTmp=imuData.roll;
    pitchTmp=-imuData.pitch;
    if(useImuYawfusedWithMag){
        yawTmp=-estiYawValue;
    }else{
        yawTmp=-imuData.yaw;
    }

    float R_NED[9];
    uavMathObj->eulertoRotation(rollTmp,pitchTmp,yawTmp,R_NED);
    if(useSonarComp){
        bodyVzFromSonar = (velFromSonarCompFilted-R_NED[6]*x(2)-R_NED[7]*x(3))/R_NED[8];
    }else if(useSonarRaw){
        bodyVzFromSonar = opticalFlowData.opticalLocal_vz;
    }
//    bodyVzFromSonar = (velFromSonarCompFilted-R_NED[6]*viconData.vx-R_NED[7]*viconData.vy)/R_NED[8];

    //cout<<"bodyVz:"<<bodyVzFromBaro<<" viconBodyVz:"<<viconData.vz<<endl;

}

void UavMotionEstimationEKF::autoAdaptEstiVel(){
    double qualityTmp=opticalFlowData.quality;
//    qualityTmp=qualHighThresh-10;
    if(averageQual<qualHighThresh){
        bodyVel[0] = x(2);
        bodyVel[1] = x(3);
//        bodyVel[2] = x(6);
        if(useSonarRaw){
            bodyVel[2] = opticalFlowData.opticalLocal_vz;
        }else if(useSonarComp){
            bodyVel[2] = bodyVzFromSonar;
        }else{
            bodyVel[2] = opticalFlowData.opticalLocal_vz;
        }

    }
    else if(averageQual>qualHighThresh){
        double flowWeight=0.5;
//        bodyVel[0] = flowWeight*opticalFlowData.opticalLocal_vx+(1-flowWeight)*x(2);
//        bodyVel[1] = flowWeight*opticalFlowData.opticalLocal_vy+(1-flowWeight)*x(3);
        bodyVel[0] = x(2);
        bodyVel[1] = x(3);
        if(useSonarRaw){
            bodyVel[2] = flowWeight*opticalFlowData.opticalLocal_vz+(1-flowWeight)*x(4);
        }else if(useSonarComp){
            bodyVel[2] = flowWeight*bodyVzFromSonar+(1-flowWeight)*x(4);
        }
        else{
            bodyVel[2] = flowWeight*opticalFlowData.opticalLocal_vz+(1-flowWeight)*x(4);
        }

    }
    if(abs(x(2))<velLowThresh && abs(x(3))<velLowThresh && averageQual<qualLowThresh){
        if(coutOnceFlag){
            //cout<<"danger state, low optical_flow quality and low velocity!"<<endl;
            coutOnceFlag=false;
        }
    }
    else{
        if(!coutOnceFlag){
            //cout<<"normal state!"<<endl;
            coutOnceFlag=true;
        }
    }

}
void UavMotionEstimationEKF::calculatePath(double dt){
    //transform NED to NWU for mrh's control
    double rollTmp,pitchTmp,yawTmp;
    rollTmp=imuData.roll;
    pitchTmp=-imuData.pitch;
    if(useImuYawfusedWithMag){
        yawTmp=-estiYawValue;
    }else{
        yawTmp=-imuData.yaw;
    }

    double bodyVx = bodyVel[0];
    double bodyVy = -bodyVel[1];
    double bodyVz = -bodyVel[2];
    float R_NWU[9];
    double bodyVelTmp[3];
    uavMathObj->eulertoRotation(rollTmp,pitchTmp,yawTmp,R_NWU);
    bodyVelTmp[0]=bodyVx;bodyVelTmp[1]=bodyVy;bodyVelTmp[2]=bodyVz;
    worldVel[0]=0;worldVel[1]=0;worldVel[2]=0;
    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
            worldVel[i] += (double)R_NWU[3*i+j]*bodyVelTmp[j];
        }
    }
    //transform from NWU to NEU for mrh...
    double worldVelNEU[3];
    worldVelNEU[0]=worldVel[0];
    worldVelNEU[1]=-worldVel[1];
    worldVelNEU[2]=worldVel[2];

    worldPoseX += worldVelNEU[0]*dt;
    worldPoseY += worldVelNEU[1]*dt;
    worldPoseZ += worldVelNEU[2]*dt;
//    worldPoseX += worldVel[0]*dt;
//    worldPoseY += worldVel[1]*dt;
//    worldPoseZ += worldVel[2]*dt;
//    cout<<"posex:"<<worldVel[0]<<' '<<"posey:"<<worldVel[1]<<' '<<"posez:"<<worldVel[2]<<endl;


    //cout<<"dt:"<<dt<<" worldVelX:"<<worldVel[0]<<" worldVelY:"<<worldVel[1]<<" worldVelZ:"<<worldVel[2]<<endl;

}

void UavMotionEstimationEKF::publishEstiBodyVel(){
    geometry_msgs::Vector3 estiBodyVel;
    //transform NED to NWU for mrh's control
    //transform NWU to NEU for mrh's control
    estiBodyVel.x=bodyVel[0];
    estiBodyVel.y=bodyVel[1];
    estiBodyVel.z=-bodyVel[2];

    //calculate the total horizonal estimation velocity
//    double estiVxDebug=bodyVel[0];
//    double estiVyDebug=bodyVel[1];
//    estiBodyVel.z = sqrt(estiVxDebug*estiVxDebug+estiVyDebug*estiVyDebug);

    estiBodyVel_pub.publish(estiBodyVel);
}
void UavMotionEstimationEKF::publishEstiRPY(){
    geometry_msgs::Vector3 estiRPY;
    estiRPY.x=worldVel[0];
    estiRPY.y=-worldVel[1];
    estiRPY.z=worldVel[2];
//    estiRPY.x=imuData.roll;
//    estiRPY.y=-imuData.pitch;
//    estiRPY.z=-estiYawValue;
//    estiRPY.x=imuData.roll;
//    estiRPY.y=imuData.pitch;
//    estiRPY.z=-imuData.yaw;
//    estiRPY.x=viconData.roll;
//    estiRPY.y=viconData.pitch;
//    estiRPY.z=viconData.yaw;

    estiRPY_pub.publish(estiRPY);
}

void UavMotionEstimationEKF::publishEstiWorldVel(){
    geometry_msgs::TwistStamped estiWorldVel;
//     geometry_msgs::Vector3 estiWorldVel;
    //transform from NWU to NEU for mrh...
    double worldVelNEU[3];
    worldVelNEU[0]=worldVel[0];
    worldVelNEU[1]=-worldVel[1];
    worldVelNEU[2]=worldVel[2];

    estiWorldVel.twist.linear.x=worldVelNEU[0];
    estiWorldVel.twist.linear.y=worldVelNEU[1];
//    estiWorldVel.twist.linear.z=worldVelNEU[2];
     //use vicon for altitude control
    estiWorldVel.twist.linear.z=-viconData.worldVz;

//    estiWorldVel.x=worldVel[0];
//    estiWorldVel.y=worldVel[1];
//    estiWorldVel.z=worldVel[2];

    estiWorldVel_pub.publish(estiWorldVel);
}

void UavMotionEstimationEKF::publishEstiWolrdPose(){
    geometry_msgs::PoseStamped worldPoseTmp;
    //transform euler angle from NED to NWU for mrh's control
    double rollTmp,pitchTmp,yawTmp;
    rollTmp=imuData.roll;
    pitchTmp=-imuData.pitch;
    if(useImuYawfusedWithMag){
        yawTmp=-estiYawValue;
    }else{
        yawTmp=-imuData.yaw;
    }
    double quaternionTmp[4];
    uavMathObj->eulertoQ(rollTmp,pitchTmp,yawTmp,quaternionTmp);
    worldPoseTmp.header.stamp = ros::Time::now();
    worldPoseTmp.pose.orientation.w = quaternionTmp[0];
    worldPoseTmp.pose.orientation.x = quaternionTmp[1];
    worldPoseTmp.pose.orientation.y = quaternionTmp[2];
    worldPoseTmp.pose.orientation.z = quaternionTmp[3];
    worldPoseTmp.pose.position.x = worldPoseX;
    worldPoseTmp.pose.position.y = worldPoseY;
//    worldPoseTmp.pose.position.z = worldPoseZ;
//    worldPoseTmp.pose.position.z = -x(5);
    worldPoseTmp.pose.position.z = - viconData.z;
    estiWorldPose_pub.publish(worldPoseTmp);
}
void UavMotionEstimationEKF::publishHeightFromSonar(){
    geometry_msgs::Vector3 heightOfSonar;
//    heightOfSonar.x = viconData.vx;
//    heightOfSonar.y = opticalFlowData.opticalLocal_vz;
//    heightOfSonar.y = imuData.acceleraRaw_x/dragForceParamX;
//    heightOfSonar.z = viconData.vz;
    heightOfSonar.x = x(5);
    heightOfSonar.y = -opticalFlowData.distance;
    heightOfSonar.z = viconData.z;
    //heightOfSonar.z = heightFromBaroKfFilted;
//    heightOfSonar.y = heightFromSonarCompFilted;
    //heightOfSonar.y = -opticalFlowData.distance;
    //heightOfSonar.x = velFromBaroCompFilted;
    //heightOfSonar.y = viconData.worldVz;
//    heightOfSonar.x = bodyVzFromSonar;
//    heightOfSonar.z = viconData.vz;
//    heightOfSonar.y = x(6);
//    heightOfSonar.x = x(0);
//    heightOfSonar.y = imuData.roll;
//    heightOfSonar.z = viconData.roll;

    heightFromSonar_pub.publish(heightOfSonar);
    //cout<<"baro:"<<heightOfSonar.x<<" vicon_z:"<<viconData.z<<endl;
}

bool UavMotionEstimationEKF::processSensorData(double timeNow){
        double t=timeNow;
        Vector3d imuAcc,bodyAngularVel;
        imuAcc(0)=imuData.accelera_x;
        imuAcc(1)=imuData.accelera_y;
        imuAcc(2)=imuData.accelera_z;
//        bodyAngularVel(0)=imuData.angularVx*180/M_PI;
//        bodyAngularVel(1)=imuData.angularVy*180/M_PI;
//        bodyAngularVel(2)=imuData.angularVz*180/M_PI;
        bodyAngularVel(0)=imuData.angularVx;
        bodyAngularVel(1)=imuData.angularVy;
        bodyAngularVel(2)=imuData.angularVz;
        this->imuAcc=imuAcc;
        this->bodyAngularVel=bodyAngularVel;

        double dt;
        if(!initialized){
            initialized=true;
            start_t=t;
            current_t=t-start_t;
        }
        else{
            dt = t-start_t-current_t;
            //************determin the measment noise cov of flow based on quality**********
            double qualityTmp=(double)opticalFlowData.quality;
//            qualityTmp=255;
            sumQual += qualityTmp;
            processCounter += 1;
            averageQual = sumQual/processCounter;
//            averageQual = 253;
            measFlowNoiseCovUpdateInterval += dt;
            if(measFlowNoiseCovUpdateInterval>1){

                if(averageQual>qualLowThresh){
//                    measurement_R(2,2)=measFlowVxNoiseCovInit*(flowQualInitial/qualityTmp);
//                    measurement_R(3,3)=measFlowVyNoiseCovInit*(flowQualInitial/qualityTmp);
                    double cor=(exp(pow(flowQualInitial/averageQual,3)-1));
                    cout<<averageQual<<' '<<"cor:"<<cor<<endl;
                    measurement_R(2,2)=measFlowVxNoiseCovInit*cor;
                    measurement_R(3,3)=measFlowVyNoiseCovInit*cor;
                }
                sumQual=0;processCounter=0;
                measFlowNoiseCovUpdateInterval=0;
            }
            //******************************************************************************
            sonarKfFilter();
            sonarCompFilter(dt);
            calcuBodyVz();
//            magKfFilter();
//            calcuYawFromMag();
//            estiYawKf(dt);
//            baroKfFilter();
//            baroCompFilter(dt);
            correct_measurement(t);
            autoAdaptEstiVel();
            calculatePath(dt);
            publishEstiBodyVel();
            publishEstiWorldVel();
            publishEstiWolrdPose();
        }

        current_t=t-start_t;

        publishEstiRPY();
        publishHeightFromSonar();


    return true;

}
