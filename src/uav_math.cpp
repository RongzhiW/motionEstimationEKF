#include "uav_math.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

/**
Euler angle defination: zyx
Rotation matrix: C_body2inertial
**/

void uav_math::eulertoQ(double roll,double pitch,double yaw,double q[]){
    double fCosHRoll = cos(roll * .5f);
    double fSinHRoll = sin(roll * .5f);
    double fCosHPitch = cos(pitch * .5f);
    double fSinHPitch = sin(pitch * .5f);
    double fCosHYaw = cos(yaw * .5f);
    double fSinHYaw = sin(yaw * .5f);

    /// Cartesian coordinate System
    q[0] = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
    q[1] = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
    q[2]= fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
    q[3] = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
    //std::cout<<"q: "<<q[0]<<' '<<q[1]<<' '<<q[2]<<' '<<q[3]<<std::endl;
}

void uav_math::qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[]){
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

void uav_math::qmultiplyq(double q1[],double q2[],double q_result[]){
    q_result[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
    q_result[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
    q_result[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
    q_result[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

void uav_math::qtoRotation(double q[],double R[]){
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

//Z-Y-X euler angle
void uav_math::eulertoRotation(float_t roll,float_t pitch,float_t yaw,float R[]){
    R[0] = cos(yaw) * cos(pitch);
    R[1] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw)*cos(roll);
    R[2] = cos(yaw) * sin(pitch) * cos(roll)+sin(yaw)*sin(roll);
    R[3] = sin(yaw) * cos(pitch);
    R[4] = sin(yaw) * sin(pitch) * sin(roll)+cos(yaw)*cos(roll);
    R[5] = sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    R[6] = -sin(pitch);
    R[7] = cos(pitch)*sin(roll);
    R[8] = cos(pitch)*cos(roll);
}


Quaterniond euler2quaternion(Vector3d euler)
{
  double cr = cos(euler(0)/2);
  double sr = sin(euler(0)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(2)/2);
  double sy = sin(euler(2)/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q;
}


Matrix3d quaternion2mat(Quaterniond q)
{
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
  return m;
}

Vector3d mat2euler(Matrix3d m)
{
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

Quaterniond mat2quaternion(Matrix3d m)
{
  //return euler2quaternion(mat2euler(m));
  Quaterniond q;
  double a, b, c, d;
  a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2))/2;
  b = (m(2, 1) - m(1, 2))/(4*a);
  c = (m(0, 2) - m(2, 0))/(4*a);
  d = (m(1, 0) - m(0, 1))/(4*a);
  q.w() = a; q.x() = b; q.y() = c; q.z() = d;
  return q;
}

Matrix3d euler2mat(Vector3d euler)
{
  double cr = cos(euler(0));
  double sr = sin(euler(0));
  double cp = cos(euler(1));
  double sp = sin(euler(1));
  double cy = cos(euler(2));
  double sy = sin(euler(2));
  Matrix3d m;
  m << cp*cy,  -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy,
       cp*sy,  cr*cy + sr*sp*sy,  -sr*cy + cr*sp*sy,
       -sp,    sr*cp,             cr*cp;
  return m;
}

Vector3d quaternion2euler(Quaterniond q)
{
  return mat2euler(quaternion2mat(q));
}
