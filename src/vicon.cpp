#include "vicon.h"

/**
  vicon pose is nwu
**/

void Vicon::setup()
{


      for(int i=1;i<4;i++){
          qWFromV[i]=0;
          qbwStart[i]=0;
          qbv[i]=0;
      }
      qWFromV[0]=1;
      qbwStart[0]=1;
      qbv[0]=1;
     //viconData_sub=nh.subscribe("/vicon/ardrone01/ardrone01", 1,&Vicon::viconDataCallback,this);
     viconData_sub=nh.subscribe("/vicon/uav_stereo02/uav_stereo02", 1,&Vicon::viconDataCallback,this);

      //viconMarkers_sub=nh.subscribe("/vicon/markers",1,&Vicon::viconMarkersCallback,this);
     uavMathObj = new uav_math();

     //*********************kf for vicon velocity*****************
     viconKF_vx=0;viconKF_vy=0;viconKF_vz=0;
     kfP=1;kfQ=0.00001;kfR=0.01;kfKk=0;
     //***********************************************************

     //*********************kf for vicon acceleration*****************
     viconKF_ax=0;viconKF_ay=0;viconKF_az=0;
     acckfP=1;acckfQ=0.00001;acckfR=0.01;acckfKk=0;
     //***********************************************************

     //****gitch numer****
     vxGitchNum=0;vyGitchNum=0;vzGitchNum=0;rollGitchNum=0;pitchGitchNum=0;yawGitchNum=0;
     ROS_INFO("Vicon Start!");
}

void Vicon::viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    double poseVicon[3],pose[3],qbw[4];
    qbv[0]=msg->transform.rotation.w;
    qbv[1]=msg->transform.rotation.x;
    qbv[2]=msg->transform.rotation.y;
    qbv[3]=msg->transform.rotation.z;

    poseVicon[0]=msg->transform.translation.x;
    poseVicon[1]=msg->transform.translation.y;
    poseVicon[2]=msg->transform.translation.z;

    if(!isViconUpdate){
        startTime=msg->header.stamp.toSec();
        frameCountStart=msg->header.seq;
        frameCountLast=frameCountStart;
        isViconUpdate=true;
        return;
    }

    frameCount=msg->header.seq;
    double dt=(frameCount-frameCountLast)*timeBetweenFrame;
    frameCountLast=frameCount;

    //***********calculate the body angular from vicon*******
    double qDotDouble[4],angularFromVicon[4],qbvConj[4];
    qDotDouble[0]=2*(qbv[0]-viconQlast[0])/dt;
    qDotDouble[1]=2*(qbv[1]-viconQlast[1])/dt;
    qDotDouble[2]=2*(qbv[2]-viconQlast[2])/dt;
    qDotDouble[3]=2*(qbv[3]-viconQlast[3])/dt;
    qbvConj[0]=qbv[0];qbvConj[1]=-qbv[1];qbvConj[2]=-qbv[2];qbvConj[3]=-qbv[3];
    uavMathObj->qmultiplyq(qbvConj,qDotDouble,angularFromVicon);
    state.bodyAngX=angularFromVicon[1];
    state.bodyAngY=angularFromVicon[2];
    state.bodyAngZ=angularFromVicon[3];
    viconQlast[0]=qbv[0];viconQlast[1]=qbv[1];viconQlast[2]=qbv[2];viconQlast[3]=qbv[3];

    //cout<<"pose_x_vicon: "<<poseVicon[0]<<" pose_y_vicon: "<<poseVicon[1]<<" pose_z_vicon: "<<poseVicon[2]<<endl;
    if(startConvert && dt>0.005){
        if(!isConvertInit){
            //conjugate
            double qConj[4];
            double rollTmp,pitchTmp,yawTmp;
            uavMathObj->qtoEuler(rollTmp,pitchTmp,yawTmp,qbv);
            uavMathObj->eulertoQ(0,0,yawTmp,qConj);
            qConj[0]=qConj[0];
            qConj[1]=-qConj[1];
            qConj[2]=-qConj[2];
            qConj[3]=-qConj[3];

            qmultiplyq(qbwStart,qConj,qWFromV);
            //Rwv rotates vectors in vicon to world coordinate
            getRotation(qWFromV,Rwv);
            cout<<"vicon_yaw_init:"<<yawTmp<<endl;
            translation[0]=poseVicon[0];
            translation[1]=poseVicon[1];
            translation[2]=poseVicon[2];
            state.x=0;
            state.y=0;
            state.z=0;
            poseLast[0]=state.x;
            poseLast[1]=state.y;
            poseLast[2]=state.z;
            isposedata=true;
            isConvertInit=true;
        }
        else{
            //qbw represents the rotation of bodyFrame in world NWU coordinate
            qmultiplyq(qWFromV,qbv,qbw);
            //gain the euler angle in NWU world coordinate
            qtoEuler(state.roll,state.pitch,state.yaw,qbw);
            for(int i=0;i<2;++i)poseVicon[i]-=translation[i];
            transformWorldFromVicon(poseVicon,pose);
            state.x=pose[0];
            state.y=pose[1];
            state.z=pose[2];

            if(isposedata){
                state.vx=(pose[0]-poseLast[0])/dt;
                state.vy=(pose[1]-poseLast[1])/dt;
                state.vz=(pose[2]-poseLast[2])/dt;

                poseLast[0]=state.x;
                poseLast[1]=state.y;
                poseLast[2]=state.z;

                //***********filter the glitch of vicon velocity***********
                if(isveldata){
                    double velGitchNumThresh=3;
                    double rpyGitchNumThresh=4;
                    double velGitchValueThresh=1.5;
                    double rpyGitchValueThresh=1.8;
                    //cout<<abs((state.roll-stateLast.roll)/dt)<<endl;
                    if(vxGitchNum<=velGitchNumThresh && abs((state.vx-stateLast.vx)/dt)>velGitchValueThresh){
                        state.vx=stateLast.vx;
                        vxGitchNum+=1;
                    }
                    else{
                        vxGitchNum=0;
                    }
                    if(vyGitchNum<=velGitchNumThresh && abs((state.vy-stateLast.vy)/dt)>velGitchValueThresh){
                        state.vy=stateLast.vy;
                        vyGitchNum+=1;
                    }
                    else{
                        vyGitchNum=0;
                    }
                    if(vzGitchNum<=velGitchNumThresh && abs((state.vz-stateLast.vz)/dt)>velGitchValueThresh){
                        state.vz=stateLast.vz;
                        vzGitchNum+=1;
                    }
                    else{
                        vzGitchNum=0;
                    }
                    if(rollGitchNum<=rpyGitchNumThresh && abs((state.roll-stateLast.roll)/dt)>rpyGitchValueThresh){
                        state.roll=stateLast.roll;
                        rollGitchNum+=1;
                    }
                    else{
                        rollGitchNum=0;
                    }
                    if(pitchGitchNum<=rpyGitchNumThresh && abs((state.pitch-stateLast.pitch)/dt)>rpyGitchValueThresh){
                        state.pitch=stateLast.pitch;
                        pitchGitchNum+=1;
                    }
                    else{
                        pitchGitchNum=0;
                    }
                    if((yawGitchNum<=rpyGitchNumThresh && abs(state.yaw-stateLast.yaw)/dt)>rpyGitchValueThresh){
                        state.yaw=stateLast.yaw;
                        yawGitchNum+=1;
                    }
                    else{
                        yawGitchNum=0;
                    }
                }
                stateLast.vx = state.vx;
                stateLast.vy = state.vy;
                stateLast.vz = state.vz;
                stateLast.roll = state.roll;
                stateLast.pitch = state.pitch;
                stateLast.yaw = state.yaw;
                //*********************************************************

                //***********KF for vicon velocity**********
//                double zk_vx,zk_vy,zk_vz;
//                zk_vx=state.vx;
//                zk_vy=state.vy;
//                zk_vz=state.vz;
//                viconKF_vx=viconKF_vx;
//                viconKF_vy=viconKF_vy;
//                viconKF_vz=viconKF_vz;
//                kfP=kfP+kfQ;
//                kfKk=kfP/(kfP+kfR);
//                viconKF_vx=viconKF_vx+kfKk*(zk_vx-viconKF_vx);
//                viconKF_vy=viconKF_vy+kfKk*(zk_vy-viconKF_vy);
//                viconKF_vz=viconKF_vz+kfKk*(zk_vz-viconKF_vz);
//                kfP=(1-kfKk)*kfP;
//                state.vx=viconKF_vx;
//                state.vy=viconKF_vy;
//                state.vz=viconKF_vz;
                //****************************************//

                state.worldVx=state.vx;
                state.worldVy=state.vy;
                state.worldVz=state.vz;

                //**convert world velocity to body velociy**
                //calculate R_NWU
                float R_NWU[9];
                uavMathObj->eulertoRotation((float)state.roll,(float)state.pitch,(float)state.yaw,R_NWU);
                //calculate body velocity
                double NWU_vel[3],bodyVel[3];
                bodyVel[0]=0;bodyVel[1]=0;bodyVel[2]=0;
                NWU_vel[0]=state.vx;NWU_vel[1]=state.vy;NWU_vel[2]=state.vz;
                for(int i=0;i<3;++i){
                    for(int j=0;j<3;++j){
                        bodyVel[i]+=R_NWU[i+j*3]*NWU_vel[j];
                    }
                }
                //state.vx is in bodyFrame
                state.vx = bodyVel[0];
                state.vy = bodyVel[1];
                state.vz = bodyVel[2];

                //*******************************************

                if(!isveldata){
//                    velLast[0]=state.worldVx;
//                    velLast[1]=state.worldVy;
//                    velLast[2]=state.worldVz;
                    velLast[0]=state.vx;
                    velLast[1]=state.vy;
                    velLast[2]=state.vz;
                }
                isveldata=true;
            }

            //state.ax is in world coordinate
            if(isveldata){
//                state.ax=(state.worldVx-velLast[0])/dt;
//                state.ay=(state.worldVy-velLast[1])/dt;
//                state.az=(state.worldVz-velLast[2])/dt;
                state.ax=(state.vx-velLast[0])/dt;
                state.ay=(state.vy-velLast[1])/dt;
                state.az=(state.vz-velLast[2])/dt;

                //***********KF for vicon acceleration**********
//                double zk_ax,zk_ay,zk_az;
//                zk_ax=state.ax;
//                zk_ay=state.ay;
//                zk_az=state.az;
//                viconKF_ax=viconKF_ax;
//                viconKF_ay=viconKF_ay;
//                viconKF_az=viconKF_az;
//                acckfP=acckfP+acckfQ;
//                acckfKk=acckfP/(acckfP+acckfR);
//                viconKF_ax=viconKF_ax+acckfKk*(zk_ax-viconKF_ax);
//                viconKF_ay=viconKF_ay+acckfKk*(zk_ay-viconKF_ay);
//                viconKF_az=viconKF_az+acckfKk*(zk_az-viconKF_az);
//                acckfP=(1-acckfKk)*acckfP;
//                state.ax=viconKF_ax;
//                state.ay=viconKF_ay;
//                state.az=viconKF_az;
                //****************************************//

//                velLast[0]=state.worldVx;
//                velLast[1]=state.worldVy;
//                velLast[2]=state.worldVz;
                velLast[0]=state.vx;
                velLast[1]=state.vy;
                velLast[2]=state.vz;

                isViconAvaible=true;

            }

        }

        //state.timeStamp=msg->header.stamp.toSec()-startTime;
        state.timeStamp=(frameCount-frameCountStart)*timeBetweenFrame;

        //double vicon_t=state.timeStamp-startTime;
        double vicon_t=(frameCount-frameCountStart)*timeBetweenFrame;
        vicon_deque.push_back(make_pair(vicon_t,state));

        //cout<<"pose_x: "<<state.x<<" pose_y: "<<state.y<<" pose_z: "<<state.z<<endl;

    }

}

//Z-Y-X Euler angle
void Vicon::qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[])
{
    float_t w,x,y,z;
    //diffrent coordinates
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];
    //cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    _roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)_pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)_pitch = asin(-1.0);
    else _pitch = asin(2 * (w * y - z * x));
    _yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void Vicon::qmultiplyq(double q1[],double q2[],double q_result[])
{
    q_result[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
    q_result[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
    q_result[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
    q_result[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

void Vicon::convert2NED(double w, double x, double y, double z)
{
    qbwStart[0]=w;
    qbwStart[1]=x;
    qbwStart[2]=y;
    qbwStart[3]=z;

    startConvert=true;
    //can't put this subscribe in setup, if put it in setup will cause conflict of read and write of 'isSetQbw'
}

//Z-Y-X Euler angle
void  Vicon::getRotation(double q[],double R[])
{
    float_t w,x,y,z;
    // coordinates in NWU
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];

    R[0]=w*w+x*x-y*y-z*z;
    R[1]=2*(x*y-w*z);
    R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);
    R[4]=w*w-x*x+y*y-z*z;
    R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);
    R[7]=2*(y*z+w*x);
    R[8]=w*w-x*x-y*y+z*z;
}

void Vicon::transformWorldFromVicon(const double a_vicon[3],double a_world[3])
{
    for(int i=0;i<3;++i) a_world[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_world[i]+=Rwv[3*i+j]*a_vicon[j];
        }
    }
}
