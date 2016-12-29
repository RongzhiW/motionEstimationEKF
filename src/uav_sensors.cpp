#include "uav_sensors.h"
#include "math.h"
using namespace std;
ofstream f_sonar("/home/tianu/Record/motionEstimationData/mediaSocUAV/sonar.txt", ios::out);
bool UavSensors::setup(){
    isImuUpdated=false;
    isFirstUpdate=true,
    isImuAvaible=false;isOpticalFlowAvaible=false;isBaroAvaible=false;isMagAvaible=false;
    isBaroFirstUpdate=true;
    flowUpdateCount=0;

    const char* px4flow_topic_name="/mavros/px4flow/raw/optical_flow_rad";
    const char* imu_topic_name="/mavros/imu/data";
    //const char* imu_topic_name="/opticalFlowSensor_msgs/imu";
    //const char* imu_topic_name="/ardrone/imu";

    px4flow_sub = nh.subscribe(px4flow_topic_name, 1, &UavSensors::flowCb,this);
    imu_sub = nh.subscribe(imu_topic_name, 1, &UavSensors::imuCb,this);
    uav_01BaroSub = nh.subscribe("/mavros/imu/atm_pressure",1,&UavSensors::uav_01BaroCb,this);
    uav_01MagSub = nh.subscribe("/mavros/imu/mag",1,&UavSensors::uav_01MagCb,this);

    uavMathObj = new uav_math();
    uavMathObj->eulertoQ(M_PI,0,0,qNED_NWU);

    accAxBias=0;accAyBias=0;accAzBias=0;
    sampleNumForBias=0;
    axGitchNum=0;ayGitchNum=0;azGitchNum=0;
    distanceGitchNum=0;

    return 1;
}

void UavSensors::flowCb(const motionEstimationEKF::optical_flow_rad &msg){
    uint32_t integrationTime_us = msg.integration_time_us;
    float integratedLocal_x_pixel = msg.integrated_x;
    float integratedLocal_y_pixel = msg.integrated_y;
    uint32_t timeDeltaDistance_us = msg.time_delta_distance_us;

    opticalFlowData.quality = msg.quality;
    opticalFlowData.timeStamp += integrationTime_us;
    opticalFlowData.distanceRaw = msg.distance;

    //***************throw out the outlier of sonar data************
//    double distanceGitchNumThresh=1;
    if(flowUpdateCount==0){
        opticalFlowData.distance = msg.distance;
        opticalFlowLastDis = msg.distance;
        opticalFlowLastLastDis = msg.distance;
        flowUpdateCount+=1;
    }
    else if(flowUpdateCount==1){
        opticalFlowData.distance = msg.distance;
        opticalFlowLastDis = msg.distance;
        flowUpdateCount+=1;
    }
    else{
//        if((msg.distance==0 || abs(msg.distance-opticalFlowLastDis)>0.5) && distanceGitchNum<distanceGitchNumThresh){
        if((msg.distance==0 || abs(msg.distance-opticalFlowLastDis)>0.5)){
            opticalFlowData.distance = 2*opticalFlowLastDis-opticalFlowLastLastDis;
            distanceGitchNum++;
        }
        else{
            opticalFlowData.distance = msg.distance;
            distanceGitchNum=0;
        }
    }
    f_sonar<<opticalFlowData.distanceRaw<<endl;
    //****************************************************************

    //*******************calculate the translation in inertial frame***********
    double yaw = imuData.yaw;
    double roll = imuData.roll;
    double pitch = imuData.pitch;
    double distance = opticalFlowData.distance;

//    //transform body flow_xy from SEU to NED
//    integratedLocal_x_pixel = -integratedLocal_x_pixel;
//    integratedLocal_y_pixel = integratedLocal_y_pixel;//bag 201611162213
    //transform body flow_xy from WSU to NED
    float exTmp;
    exTmp=integratedLocal_x_pixel;
    integratedLocal_x_pixel = -integratedLocal_y_pixel;
    integratedLocal_y_pixel = -exTmp;//bag 201612131519

    //optical flow velocity in world NED
    if(integrationTime_us>0){
        opticalFlowData.opticalLocal_vx = integratedLocal_x_pixel * distance / integrationTime_us * 1000000;
        opticalFlowData.opticalLocal_vy = integratedLocal_y_pixel * distance / integrationTime_us * 1000000;
        //optical_flow from mavros, distance is in world frame
        opticalFlowData.opticalGlobal_vz = -(distance-opticalFlowLastDis)/integrationTime_us * 1000000;
//        opticalFlowData.opticalLocal_vz = -(distance-opticalFlowLastDis)/integrationTime_us * 1000000;

        //calculate R_NED
        float R_NED[9];
        uavMathObj->eulertoRotation((float)roll,(float)pitch,(float)yaw,R_NED);
        double bodyVx = opticalFlowData.opticalLocal_vx;
        double bodyVy = opticalFlowData.opticalLocal_vy;
        double worldVz = opticalFlowData.opticalGlobal_vz;
        opticalFlowData.opticalLocal_vz = (worldVz-R_NED[6]*bodyVx-R_NED[7]*bodyVy)/R_NED[8];
        //calculate world velocity
        double NED_vel[3],bodyVel[3];
        bodyVel[0]=opticalFlowData.opticalLocal_vx;
        bodyVel[1]=opticalFlowData.opticalLocal_vy;
        bodyVel[2]=opticalFlowData.opticalLocal_vz;
        NED_vel[0]=0;NED_vel[1]=0;NED_vel[2]=0;
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                NED_vel[i]+=R_NED[3*i+j]*bodyVel[j];
            }
        }
        opticalFlowData.opticalGlobal_vx = NED_vel[0];
        opticalFlowData.opticalGlobal_vy = NED_vel[1];
        opticalFlowData.opticalGlobal_vz = NED_vel[2];

        isOpticalFlowAvaible=true;
        opticalFlowLastLastDis=opticalFlowLastDis;
        opticalFlowLastDis = distance;

    }
    //***************************************************************************

    //optical flow integration path in world NED
    opticalFlowData.opticalGlobal_x_m += opticalFlowData.opticalGlobal_vx * integrationTime_us/1000000;
    opticalFlowData.opticalGlobal_y_m += opticalFlowData.opticalGlobal_vy * integrationTime_us/1000000;

    //cout <<"x: "<<opticalFlowData.opticalGlobal_x_m<<" y: "<<opticalFlowData.opticalGlobal_y_m<< endl;

}

void UavSensors::imuCb(const sensor_msgs::Imu &msg){
    //for uav2.0 mpu9250 initial rpy=(0.05 -0.14 0);initial acc=(-1,372 -0.49 -9.8);
    //q=[0.932697 0.0479 0.05667 -0.3529]

    double x = msg.orientation.x;
    double y = msg.orientation.y;
    double z = msg.orientation.z;
    double w = msg.orientation.w;

    imuData.x=x;
    imuData.y=y;
    imuData.z=z;
    imuData.w=w;

    //%%%%%%%%%%%%%%%%%%%Quaternion to Matrix%%%%%%%%%%%%%%%%%%%%%%%%%
    double x2 = x*x;
    double y2 = y*y;
    double z2 = z*z;
    double xy = x*y;
    double xz = x*z;
    double yz = y*z;
    double wx = w*x;
    double wy = w*y;
    double wz = w*z;

    R11 = 1-2*(y2+z2), R12 = 2*(xy-wz), R13 = 2*(xz+wy);
    R21 = 2*(xy+wz), R22 = 1-2*(x2+z2), R23 = 2*(yz-wx);
    R31 = 2*(xz-wy), R32 = 2*(yz+wx), R33 = 1-2*(x2+y2);

    //raw attitude in NWU Z-Y-X mode
    double rollNWU = atan2(2*w*x+2*y*z,1-2*x*x-2*y*y);
    double pitchNWU = asin(2*w*y-2*z*x);
    double yawNWU = atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);

    //raw acceleration in NWU
    double acc_xNWU = msg.linear_acceleration.x;
    double acc_yNWU = msg.linear_acceleration.y;
    double acc_zNWU = msg.linear_acceleration.z;

    //raw angular rate in NWU
    double ang_xNWU = msg.angular_velocity.x;
    double ang_yNWU = msg.angular_velocity.y;
    double ang_zNWU = msg.angular_velocity.z;

    //**************calibrate the inital inclination*****************
//    double qAcc[4],qAngular[4];
//    qAcc[0]=0;qAcc[1]=acc_xNWU;qAcc[2]=acc_yNWU;qAcc[3]=acc_zNWU;
//    qAngular[0]=0;qAngular[1]=ang_xNWU;qAngular[2]=ang_yNWU;qAngular[3]=ang_zNWU;
//    //qAcc[0]=accInit[0];qAcc[1]=accInit[1];qAcc[2]=accInit[2];qAcc[3]=accInit[3];
//    double qResult[4];
//    uavMathObj->qmultiplyq(qInit,qAcc,qResult);
//    uavMathObj->qmultiplyq(qResult,qInitConju,qAcc);
//    acc_xNWU=qAcc[1];
//    acc_yNWU=qAcc[2];
//    acc_zNWU=qAcc[3];

//    uavMathObj->qmultiplyq(qInit,qAngular,qResult);
//    uavMathObj->qmultiplyq(qResult,qInitConju,qAngular);
//    ang_xNWU=qAngular[1];
//    ang_yNWU=qAngular[2];
//    ang_zNWU=qAngular[3];
    //cout<<"qAcc0:"<<qAcc[0]<<" qAcc1:"<<qAcc[1]<<" qAcc2:"<<qAcc[2]<<" qAcc:"<<qAcc[3]<<endl;
    //*****************************************************************

    //***transform accelerometer value from NWU to NED
    imuData.accelera_x = acc_xNWU;
    imuData.accelera_y = -acc_yNWU;
    imuData.accelera_z = -acc_zNWU;

    //***transform angular velocit from NWU to NED
    imuData.angularVx = ang_xNWU;
    imuData.angularVy = -ang_yNWU;
    imuData.angularVz = -ang_zNWU;

    //***transform attitude from NWU to NED
    imuData.roll = rollNWU;
    imuData.pitch = -pitchNWU;
    imuData.yaw = -yawNWU;
    imuYawRaw = -yawNWU;

    //***********calculate the acc bias**********
    double sampleNumForBiasThresh=20;
    if(sampleNumForBias<sampleNumForBiasThresh){
        accAxBias += imuData.accelera_x;
        accAyBias += imuData.accelera_y;
        accAzBias += imuData.accelera_z;
    }
    sampleNumForBias += 1;
    if(sampleNumForBias==sampleNumForBiasThresh){
        accAxBias /= sampleNumForBiasThresh;
        accAyBias /=sampleNumForBiasThresh;
        accAzBias /=sampleNumForBiasThresh;
        accAzBias += 9.8;
        //tell others imu has data
        isImuUpdated=true;
        imuDataLast.accelera_x=imuData.accelera_x-accAxBias;
        imuDataLast.accelera_y=imuData.accelera_y-accAyBias;
        imuDataLast.accelera_z=imuData.accelera_z-accAzBias;
        cout<<"accAxBias:"<<accAxBias<<' '<<"accAyBias:"<<accAyBias<<' '<<"accAzBias:"<<accAzBias<<endl;
    }
//    isImuUpdated=true;
    if(isImuUpdated){
        imuData.accelera_x -= accAxBias;
        imuData.accelera_y -= accAyBias;
        imuData.accelera_z -= accAzBias;

        //***********filter the glitch of imu accelerometer***********
        double axGitchNumThresh=3;
        double ayGitchNumThresh=3;
        double azGitchNumThresh=3;
        double axGitchValueThresh=0.5;
        double ayGitchValueThresh=0.5;
        double azGitchValueThresh=0.5;

        if(axGitchNum<=axGitchNumThresh && abs((imuData.accelera_x-imuDataLast.accelera_x))>axGitchValueThresh){
            imuData.accelera_x=imuDataLast.accelera_x;
            axGitchNum+=1;
        }
        else{
            axGitchNum=0;
        }

        if(ayGitchNum<=ayGitchNumThresh && abs((imuData.accelera_y-imuDataLast.accelera_y))>ayGitchValueThresh){
            imuData.accelera_y=imuDataLast.accelera_y;
            ayGitchNum+=1;
        }
        else{
            ayGitchNum=0;
        }

        if(azGitchNum<=azGitchNumThresh && abs((imuData.accelera_z-imuDataLast.accelera_z))>azGitchValueThresh){
            imuData.accelera_z=imuDataLast.accelera_z;
            azGitchNum+=1;
        }
        else{
            azGitchNum=0;
        }
        imuDataLast.accelera_x=imuData.accelera_x;
        imuDataLast.accelera_y=imuData.accelera_y;
        imuDataLast.accelera_z=imuData.accelera_z;
        //*********************kf for imu******************
        //acc_X
        AccKfFilted[0] = AccKfFilted[0];
        imuAccKfP[0] = imuAccKfP[0]+imuAccKfQ[0];
        imuAccKfK[0] = imuAccKfP[0]/(imuAccKfP[0]+imuAccKfR[0]);
        double imuAccKfZk_x = imuData.accelera_x;
        AccKfFilted[0] = AccKfFilted[0]+imuAccKfK[0]*(imuAccKfZk_x-AccKfFilted[0]);
        imuAccKfP[0] = (1-imuAccKfK[0])*imuAccKfP[0];
        //acc_Y
        AccKfFilted[1] = AccKfFilted[1];
        imuAccKfP[1] = imuAccKfP[1]+imuAccKfQ[1];
        imuAccKfK[1] = imuAccKfP[1]/(imuAccKfP[1]+imuAccKfR[1]);
        double imuAccKfZk_y = imuData.accelera_y;
        AccKfFilted[1] = AccKfFilted[1]+imuAccKfK[1]*(imuAccKfZk_y-AccKfFilted[1]);
        imuAccKfP[1] = (1-imuAccKfK[1])*imuAccKfP[1];
        //acc_Z
        AccKfFilted[2] = AccKfFilted[2];
        imuAccKfP[2] = imuAccKfP[2]+imuAccKfQ[2];
        imuAccKfK[2] = imuAccKfP[2]/(imuAccKfP[2]+imuAccKfR[2]);
        double imuAccKfZk_z = imuData.accelera_z;
        AccKfFilted[2] = AccKfFilted[2]+imuAccKfK[2]*(imuAccKfZk_z-AccKfFilted[2]);
        imuAccKfP[2] = (1-imuAccKfK[2])*imuAccKfP[2];

        imuData.acceleraRaw_x = imuData.accelera_x;
        imuData.acceleraRaw_y = imuData.accelera_y;
        imuData.acceleraRaw_z = imuData.accelera_z;

        imuData.accelera_x = AccKfFilted[0];
        imuData.accelera_y = AccKfFilted[1];
        imuData.accelera_z = AccKfFilted[2];
        //*************************************************
        //the NED world coordinate is tha same as the initial body NED frame
        if(isFirstUpdate){
            isFirstUpdate=false;
            startTime=msg.header.stamp.toSec();
            //calculate q in NWU
            uavMathObj->eulertoQ(imuData.roll,-imuData.pitch,-imuData.yaw,qObj0);
            cout<<"imu roll:"<<imuData.roll<<" pitch:"<<-imuData.pitch<<" yaw:"<<-imuData.yaw<<endl;
            //give the q in NWU world, start to convert the vicon coordinate to NWU world coordinate
            viconObj->convert2NED(qObj0[0],qObj0[1],qObj0[2],qObj0[3]);
            isImuAvaible=true;
        }
        else{
            double t = msg.header.stamp.toSec()-startTime;
            imuData.timeStamp = t;
            imuDeque.push_back(make_pair(t, imuData) );
            //cout<<"imu_time: "<<t<<endl;
        }

    }

    //cout<<"wX: "<<imuData.acceleraRaw_x<<" accY: "<<imuData.acceleraRaw_y<<" accZ: "<<imuData.acceleraRaw_z<<endl;

}

void UavSensors::uav_01BaroCb(const sensor_msgs::FluidPressure &msg){
    if(isBaroFirstUpdate){
        if(msg.fluid_pressure>100000){
            uav_01BaroStart=msg.fluid_pressure;
            uav_01HeightFromBaro=0;
            uav_01HeightFromBaroLast=0;
            isBaroFirstUpdate=false;
            //cout<<"start pressure:"<<uav_01BaroStart<<endl;
        }

    }
    else{
        uav_01Baro = msg.fluid_pressure;
        uav_01HeightFromBaro = -8724*(1+25/273.0)*log(uav_01Baro*1.0/uav_01BaroStart);
        if(abs(uav_01HeightFromBaro)>10)uav_01HeightFromBaro=uav_01HeightFromBaroLast;
        uav_01HeightFromBaroLast = uav_01HeightFromBaro;
        isBaroAvaible=true;
    }
    //cout<<"pressure:"<<uav_01Baro<<" height:"<<uav_01HeightFromBaro<<endl;
}

void UavSensors::uav_01MagCb(const sensor_msgs::MagneticField &msg){
    uav_01Mag.x=msg.magnetic_field.x;
    uav_01Mag.y=msg.magnetic_field.y;
    uav_01Mag.z=msg.magnetic_field.z;
    isMagAvaible=true;
}
