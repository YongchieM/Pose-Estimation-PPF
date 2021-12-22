#ifndef __MYPPF_PPF_MATCH_HPP__
#define __MYPPF_PPF_MATCH_HPP__

#include <opencv2/core.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <vector>
#include <unordered_map>

#include "pose_3d.hpp"

namespace myppf
{
    
typedef uint KeyType; //hash key
//node in the hash table  
typedef struct THash
{
  int ppfInd,i;
  double alpha;
  double voteVal;
} THash;
  
class PPFDetector
{
public:
  
PPFDetector(const std::string &strSettingsFile);
  
virtual ~PPFDetector();

//set parameters for clustering  
void setClusterParams(const double positionThreshold=-1, const double rotationThreshold=-1, const bool useWeightedClustering=false);
//train the model
//input: path of the CAD model  
void trainModel(const std::string &model_path);
/*set the scene for matching
input:
empty_scene: path of the empty scene, ply file
scene_pc:    path of the real scene, ply file
model_path:  path of the CAD model
*/
void setScene(const std::string& empty_scene, const std::string& scene_pc, const std::string& model_path);

void match();
void rescore(); //rescore the poses
void rescoreAfterICP(); //perform icp for some good poses, then rescore
Pose3DPtr getPoseAfterICP(); //perform icp to get the best pose

void save_model(const char* file);
void load_model(const char* file);

void save_poses(const char* file);
void load_poses(const char* file);

void test();

//this function is for finding the most similar gear
int getScore(Pose3DPtr pose, const char* model, const char* scene, const float threshold=2.5);

protected:
cv::FileStorage parameters; //parameters

float angle_step, distance_step; //angle step and distance step when calculating the key value for point-pair feature
float angle_step_relative, sampling_step_relative; //parameters for calculating angle step and distance step
int scene_skip_length; //step size when chosing the first point in the scene point cloud
float model_diameter;

std::vector<Pose3DPtr> poses;
pcl::PointCloud<pcl::PointNormal>::Ptr sampled_scene; //down-sampled scene point cloud
pcl::PointCloud<pcl::PointNormal>::Ptr sampled_pc; //down-sampled model point cloud
pcl::PointNormal obj_centre;
int num_ref_points; //number of reference points in training stage

std::unordered_map<KeyType,std::vector<THash*>> hash_table; //hash table including the ppf information of the model
THash* hash_nodes;

double position_threshold, rotation_threshold; //parameters for clustering
bool use_weighted_avg;

void clearTrainedModel();
  
private:

//compute point-pair features
void computePPFFeatures(const Eigen::Vector3f& p1, const Eigen::Vector3f& n1, const Eigen::Vector3f& p2, const Eigen::Vector3f& n2, Eigen::Vector4f& f);
//hash keys for neighbors of a given point-pair feature
std::vector<KeyType> searchNeighbors(const Eigen::Vector4f& f, const double AngleStep, const double DistanceStep);
//neighbors of a given point-pair feature
Eigen::Vector4i neighborInd(const Eigen::Vector4f& f, const double AngleStep, const double DistanceStep);

//if two poses are in the same cluster
bool matchPose(const Pose3D& sourcePose, const Pose3D& targetPose);
//cluster poses
void clusterPoses(std::vector<Pose3DPtr>& poseList, int numPoses, std::vector<Pose3DPtr>& finalPoses);

bool trained;
};
  
}

#endif
