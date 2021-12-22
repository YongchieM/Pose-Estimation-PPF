#ifndef __MYPPF_UTILS_HPP_
#define __MYPPF_UTILS_HPP_

#include<cmath>
#include<cstdio>
#include<eigen3/Eigen/Dense>
#include<pcl/point_types.h>

namespace myppf
{

const float EPS=1.192092896e-07F;

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif


static inline void TNormalize3(Eigen::Vector3d& v)
{
    double norm = v.norm();
    if(norm>EPS)
    {
        v /= norm;
        
    }
}

static inline void TNormalize3(Eigen::Vector3f& v)
{
    float norm = v.norm();
    if(norm>EPS)
    {
        v /= norm;
        
    }
}

static inline double TAngle3Normalized(const Eigen::Vector3f& a, const Eigen::Vector3f& b)
{
    return acos(a.dot(b));
}

static inline double TAngle3Normalized(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return acos(a.dot(b));
}

static inline void aaToR(const Eigen::Vector3d& axis, double angle, Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(angle, axis);
    R = aa.toRotationMatrix();
}

static inline void getUnitXRotation(double angle, Eigen::Matrix3d& Rx)
{
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
    Rx = aa.toRotationMatrix();
}

static inline void getUnitYRotation(double angle, Eigen::Matrix3d& Ry)
{
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    Ry = aa.toRotationMatrix();
}

static inline void getUnitZRotation(double angle, Eigen::Matrix3d& Rz)
{
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    Rz = aa.toRotationMatrix();
}

static inline void eulertoR(const Eigen::Vector3d& euler, Eigen::Matrix3d& R)
{
    Eigen::Matrix3d Rx,Ry,Rz;
    
    getUnitXRotation(euler[0], Rx);
    getUnitYRotation(euler[1], Ry);
    getUnitZRotation(euler[2], Rz);
    
    R = Rx*(Ry*Rz);
}

static inline void computeTransformRT(const Eigen::Vector3f& p1, const Eigen::Vector3f& n1, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    double angle = acos(n1[0]);
    
    Eigen::Vector3d axis(0, n1[2], -n1[1]);
    
    //try to project on the ground plane but it's already parallel
    if(n1[1]==0&&n1[2]==0)
    {
        axis[1] = 1;
        axis[2] = 0;
    }
    else
    {
        TNormalize3(axis);
    }
    
    Eigen::AngleAxisd aa(angle,axis);
    R = aa.toRotationMatrix();
    //aaToR(axis, angle, R);
    t = -R*p1.cast<double>();
}

//flip a normal to the viewing direction
static inline void flipNormalViewpoint(const Eigen::Vector3d& point, const Eigen::Vector3d& vp, Eigen::Vector3d& n)
{
    Eigen::Vector3d diff = vp-point;
    double cos_theta = diff.dot(n);
    
    if(cos_theta<0)
        n *= -1;
}

static inline void flipNormalViewpoint(const Eigen::Vector3d& vp, pcl::PointNormal& point)
{
    Eigen::Vector3d p(point.x,point.y,point.z);
    Eigen::Vector3d n(point.normal_x,point.normal_y,point.normal_z);
    
    flipNormalViewpoint(p,vp,n);
    point.normal_x = n[0];
    point.normal_y = n[1];
    point.normal_z = n[2];
}




}
#endif
