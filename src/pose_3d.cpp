#include "../include/myppf/headers.hpp"

namespace myppf
{

void Pose3D::updatePose(Eigen::Isometry3d& NewPose)
{
    this->pose = NewPose;
    
    Eigen::Matrix3d R = pose.rotation();
    q = R;
    t = pose.translation();
    
    const double trace = R.trace();
    if(fabs(trace-3)<=EPS)
    {
        angle = 0;
    }
    else
    {
        if(fabs(trace+1)<=EPS)
        {
            angle = M_PI;
        }
        else
        {
            angle = (acos((trace-1)/2));
        }
    }
}

void Pose3D::updatePose(Eigen::Matrix3d& NewR, Eigen::Vector3d& NewT)
{
    pose.matrix().block<3,3>(0,0) = NewR;
    pose.matrix().topRightCorner<3,1>() = NewT;
    
    t = NewT;
    //Eigen::Quaterniond NewQ(NewR);
    q = NewR;
    
    const double trace = NewR.trace();
    if(fabs(trace-3)<=EPS)
    {
        angle = 0;
    }
    else
    {
        if(fabs(trace+1)<=EPS)
        {
            angle = M_PI;
        }
        else
        {
            angle = (acos((trace-1)/2));
        }
    }
}

void Pose3D::updatePoseQuat(Eigen::Quaterniond& Q, Eigen::Vector3d& NewT)
{
    q = Q;
    t = NewT;
    
    Eigen::Matrix3d R = q.matrix();
    pose.matrix().block<3,3>(0,0) = R;
    pose.matrix().topRightCorner<3,1>() = t;
    
    const double trace = R.trace();
    if(fabs(trace-3)<=EPS)
    {
        angle = 0;
    }
    else
    {
        if(fabs(trace+1)<=EPS)
        {
            angle = M_PI;
        }
        else
        {
            angle = (acos((trace-1)/2));
        }
    }
}

void Pose3D::appendPose(Eigen::Isometry3d& IncrementalPose)
{
    Eigen::Matrix3d R;
    Eigen::Isometry3d PoseFull = IncrementalPose*this->pose;
    
    R = PoseFull.rotation();
    t = PoseFull.translation();
    //Eigen::Quaterniond Q(R);
    q = R;
    
    const double trace = R.trace();
    if(fabs(trace-3)<=EPS)
    {
        angle = 0;
    }
    else
    {
        if(fabs(trace+1)<=EPS)
        {
            angle = M_PI;
        }
        else
        {
            angle = (acos((trace-1)/2));
        }
    }
    
    pose = PoseFull;
}

Pose3DPtr Pose3D::clone()
{
    cv::Ptr<Pose3D> new_pose(new Pose3D(alpha,modelIndex,numVotes));
    new_pose->residual = residual;
    
    new_pose->pose = this->pose;
    new_pose->q = q;
    new_pose->t = t;
    new_pose->angle = angle;
    
    return new_pose;
}

void Pose3D::printPose()
{
    printf("\n--Pose to modelIndex %d:NumVotes=%f,Residual=%f\n",(uint)this->modelIndex,this->numVotes,this->residual);
    //Eigen::Isometry3d T = this->pose;
    std::cout<<pose.matrix()<<std::endl;
}
    
int Pose3D::writePose(FILE* f)
{
    int POSE_MAGIC = 7673;
    Eigen::Vector4d Q(q.w(),q.x(),q.y(),q.z());
    fwrite(&POSE_MAGIC, sizeof(int), 1, f);
    fwrite(&angle, sizeof(double), 1, f);
    fwrite(&numVotes, sizeof(double), 1, f);
    fwrite(&modelIndex, sizeof(int), 1, f);
    fwrite(pose.matrix().data(), sizeof(double)*16, 1, f);
    fwrite(t.data(), sizeof(double)*3, 1, f);
    fwrite(Q.data(), sizeof(double)*4, 1, f);
    fwrite(&residual, sizeof(double), 1, f);
    return 0;
}

int Pose3D::readPose(FILE *f)
{
    int POSE_MAGIC = 7673, magic;
    Eigen::Vector4d Q;

    size_t status = fread(&magic, sizeof(int), 1, f);
    if (status && magic == POSE_MAGIC)
    {
    status = fread(&angle, sizeof(double), 1, f);
    status = fread(&numVotes, sizeof(double), 1, f);
    status = fread(&modelIndex, sizeof(int), 1, f);
    status = fread(pose.matrix().data(), sizeof(double)*16, 1, f);
    status = fread(t.data(), sizeof(double)*3, 1, f);
    status = fread(Q.data(), sizeof(double)*4, 1, f);
    q.w() = Q[0];
    q.x() = Q[1];
    q.y() = Q[2];
    q.z() = Q[3];
    status = fread(&residual, sizeof(double), 1, f);
    return 0;
    }
    return -1;
}

int Pose3D::writePose(const std::string& FileName)
{
    FILE* f = fopen(FileName.c_str(),"wb");
    
    if(!f)
        return -1;
    
    int status = writePose(f);
    
    fclose(f);
    return status;
}

int Pose3D::readPose(const std::string& FileName)
{
    FILE* f = fopen(FileName.c_str(),"rb");
    
    if(!f)
        return -1;
    
    int status = readPose(f);
    
    fclose(f);
    return status;
}

void PoseCluster3D::addPose(Pose3DPtr newPose)
{
    poseList.push_back(newPose);
    this->numVotes += newPose->numVotes;
};

int PoseCluster3D::writePoseCluster(FILE *f)
{
    int POSE_CLUSTER_MAGIC = 8462597;
    fwrite(&POSE_CLUSTER_MAGIC,sizeof(int),1,f);
    fwrite(&id, sizeof(int), 1, f);
    fwrite(&numVotes, sizeof(double), 1, f);
    
    int numPoses = (int)poseList.size();
    fwrite(&numPoses, sizeof(int), 1, f);
    
    for(int i=0; i<numPoses; i++)
        poseList[i]->writePose(f);
    
    return 0;
}

int PoseCluster3D::readPoseCluster(FILE*f)
{
    int POSE_CLUSTER_MAGIC = 8462597;
    int magic=0, numPoses=0;
    size_t status;
    status = fread(&magic, sizeof(int), 1, f);
    
    if(!status||magic!=POSE_CLUSTER_MAGIC)
        return -1;
    
    status = fread(&id, sizeof(int), 1, f);
    status = fread(&numVotes, sizeof(double), 1, f);
    status = fread(&numPoses, sizeof(int), 1, f);
    fclose(f);
    
    poseList.clear();
    poseList.resize(numPoses);
    for(size_t i=0; i<poseList.size(); i++)
    {
        poseList[i] = Pose3DPtr(new Pose3D());
        poseList[i]->readPose(f);
    }
    
    return 0;
}

int PoseCluster3D::writePoseCluster(const std::string& FileName)
{
    FILE* f = fopen(FileName.c_str(), "wb");
    
    if(!f)
        return -1;
    
    int status = writePoseCluster(f);
    
    fclose(f);
    return status;
}

int PoseCluster3D::readPoseCluster(const std::string& FileName)
{
    FILE* f = fopen(FileName.c_str(), "rb");
    
    if(!f)
        return -1;
    
    int status = readPoseCluster(f);
    
    fclose(f);
    return status;
}


}
