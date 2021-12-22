#ifndef __MYPPF_POSE3D_HPP__
#define __MYPPF_POSE3D_HPP__

#include "opencv2/core/cvstd.hpp"
#include<vector>
#include<string>
#include<eigen3/Eigen/Dense>

namespace myppf
{

class Pose3D;
typedef cv::Ptr<Pose3D> Pose3DPtr;
class PoseCluster3D;
typedef cv::Ptr<PoseCluster3D> PoseCluster3DPtr;

class Pose3D
{
public:
double alpha, residual;
double numVotes;
size_t modelIndex;
    
Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
double angle;
Eigen::Vector3d t; //translation
Eigen::Quaterniond q; //quaternion

public:
Pose3D()
{
    alpha=0.0;
    t = Eigen::Vector3d::Zero();
    modelIndex=0;
    numVotes=0;
    residual=0;
}

Pose3D(double Alpha, size_t ModelIndex=0, double NumVotes=0)
{
    alpha=Alpha;
    modelIndex=ModelIndex;
    numVotes=NumVotes;
    residual=0;
}

void updatePose(Eigen::Isometry3d& NewPose);
void updatePose(Eigen::Matrix3d& NewR, Eigen::Vector3d& NewT);
void updatePoseQuat(Eigen::Quaterniond& Q, Eigen::Vector3d& NewT);
void appendPose(Eigen::Isometry3d& IncrementalPose);
void printPose();
Pose3DPtr clone();

int writePose(FILE* f);
int readPose(FILE* f);
int writePose(const std::string& FileName);
int readPose(const std::string& FileName);

virtual ~Pose3D(){}
};

class PoseCluster3D{
 
public:
std::vector<Pose3DPtr> poseList;
double numVotes;
int id;

public:
PoseCluster3D()
{
    numVotes=0;
    id=0;
}

PoseCluster3D(Pose3DPtr newPose)
{
    poseList.clear();
    poseList.push_back(newPose);
    numVotes=newPose->numVotes;
    id=0;
}

PoseCluster3D(Pose3DPtr newPose, int newId)
{
    poseList.push_back(newPose);
    this->numVotes=newPose->numVotes;
    this->id=newId;
}

virtual ~PoseCluster3D(){}

void addPose(Pose3DPtr newPose);

int writePoseCluster(FILE* f);
int readPoseCluster(FILE* f);
int writePoseCluster(const std::string& FileName);
int readPoseCluster(const std::string& FileName);
};


}

#endif

