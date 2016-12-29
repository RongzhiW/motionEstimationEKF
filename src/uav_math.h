#ifndef UAV_MATH_H
#define UAV_MATH_H
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <Eigen/Geometry>
//#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
class uav_math
{
public:
    uav_math(){}
    ~uav_math(){}

public:
   void qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[]);
   void eulertoQ(double roll,double pitch,double yaw,double q[]);
   void qtoRotation(double q[],double R[]);
   void eulertoRotation(float_t roll,float_t pitch,float_t yaw,float R[]);
   void qmultiplyq(double q1[],double q2[],double q_result[]);
   Quaterniond euler2quaternion(Vector3d euler);
   Matrix3d quaternion2mat(Quaterniond q);
   Vector3d mat2euler(Matrix3d m);
   Quaterniond mat2quaternion(Matrix3d m);
   Matrix3d euler2mat(Vector3d euler);
   Vector3d quaternion2euler(Quaterniond q);
};

#endif // UAV_MATH_H
