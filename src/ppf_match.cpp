#include "../include/myppf/headers.hpp"
#include "../include/myppf/hash_murmur64.hpp"

namespace myppf
{
  
static bool sortPoses(const Pose3DPtr& a, const Pose3DPtr& b)
{
  CV_Assert(!a.empty() && !b.empty());
  return (a->numVotes > b->numVotes);
}
  
static bool sortPoseClusters(const PoseCluster3DPtr& a, const PoseCluster3DPtr& b)
{
  CV_Assert(!a.empty() && !b.empty());
  return (a->numVotes > b->numVotes);
}

static bool sortRescore(const std::pair<int,Pose3DPtr>& a, const std::pair<int,Pose3DPtr>& b)
{
    return (a.first > b.first);
}
  
static KeyType hashPPF(const Eigen::Vector4f& f, const double AngleStep, const double DistanceStep)
{
  Eigen::Vector4i key((int)(f[0]/AngleStep), (int)(f[1]/AngleStep), (int)(f[2]/AngleStep), (int)(f[3]/DistanceStep));
  KeyType hashKey[2] = {0,0};
  
  hashMurmurx64(key.data(),4*sizeof(int),42,&hashKey[0]);
  return hashKey[0];
}

static KeyType hashPPF(const Eigen::Vector4i& key)
{
  KeyType hashKey[2] = {0,0};
  hashMurmurx64(key.data(),4*sizeof(int),42,&hashKey[0]);
  return hashKey[0];
}
  
static double computeAlpha(const Eigen::Vector3f& p1, const Eigen::Vector3f& n1, const Eigen::Vector3f& p2)
{
  Eigen::Vector3d Tmg, mpt;
  Eigen::Matrix3d R;
  double alpha;
  
  computeTransformRT(p1,n1,R,Tmg);
  mpt = Tmg + R*p2.cast<double>();
  alpha = atan2(-mpt[2], mpt[1]);
  
  if(sin(alpha)*mpt[2]>0.0)
    alpha = -alpha;
  return alpha;
}

PPFDetector::PPFDetector(const std::string &strSettingsFile)
{
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  parameters = fsSettings;

  float positionThreshold = parameters["Cluster.Position.Threshold"];
  float rotationThreshold = parameters["Cluster.Rotation.Threshold"];

  scene_skip_length = parameters["Scene.Skip.Length"];

  sampling_step_relative = parameters["Sampling.Step.Relative"];
  angle_step_relative = parameters["Angle.Step.Relative"];
  angle_step = (360.0/angle_step_relative)*M_PI/180.0;

  trained = false;

  hash_table.clear();
  hash_nodes = NULL;

  setClusterParams(positionThreshold,rotationThreshold,true);
}

void PPFDetector::setClusterParams(const double positionThreshold, const double rotationThreshold, const bool useWeightedClustering)
{
  if (positionThreshold<0)
    position_threshold = sampling_step_relative;
  else
    position_threshold = positionThreshold;

  if (rotationThreshold<0)
    rotation_threshold = ((360/angle_step) / 180.0 * M_PI);
  else
    rotation_threshold = rotationThreshold;

  use_weighted_avg = useWeightedClustering;
}

void PPFDetector::clearTrainedModel()
{
  if(this->hash_nodes)
  {
    free(this->hash_nodes);
    this->hash_nodes = 0;
  }
  
  if(!hash_table.empty())
  {
    hash_table.clear();
  }
}

//compute the PPF
void PPFDetector::computePPFFeatures(const Eigen::Vector3f& p1, const Eigen::Vector3f& n1, const Eigen::Vector3f& p2, const Eigen::Vector3f& n2, Eigen::Vector4f& f)
{
  Eigen::Vector3f d(p2-p1);
  f[3] = d.norm();
  if (f[3] <= EPS)
    return;
  d /= f[3];

  f[0] = TAngle3Normalized(n1, d);
  f[1] = TAngle3Normalized(n2, d);
  f[2] = TAngle3Normalized(n1, n2);
}

Eigen::Vector4i PPFDetector::neighborInd(const Eigen::Vector4f& f, const double AngleStep, const double DistanceStep)
{
  Eigen::Vector4i ind;
  Eigen::Vector4f eq;
  eq[0] = (float)(f[0]/AngleStep-floor(f[0]/AngleStep));
  eq[1] = (float)(f[1]/AngleStep-floor(f[1]/AngleStep));
  eq[2] = (float)(f[2]/AngleStep-floor(f[2]/AngleStep));
  eq[3] = (float)(f[3]/DistanceStep-floor(f[3]/DistanceStep));
  for(int i=0; i<4; i++)
  {
    if(eq[i]<(float)(1.0/3.0))
      ind[i] = -1;
    else if(eq[i]>(float)(2.0/3.0))
      ind[i] = 1;
    else 
      ind[i] = 0;
  }
  return ind;
}
  
std::vector<KeyType> PPFDetector::searchNeighbors(const Eigen::Vector4f& f, const double AngleStep, const double DistanceStep)
{
  std::vector<KeyType> neighbors;
  Eigen::Vector4i ind = neighborInd(f,AngleStep,DistanceStep);
  std::vector<std::vector<int>> vec;
  for(int i=0; i<4; i++)
  {
    if(ind[i] == 0)
      vec.push_back({0});
    else
      vec.push_back({0,ind[i]});
  }
  for(int v0:vec[0])
  {
    for(int v1:vec[1])
    {
      for(int v2:vec[2])
      {
        for(int v3:vec[3])
        {
          Eigen::Vector4i key((int)(f[0]/AngleStep)+v0, (int)(f[1]/AngleStep)+v1, (int)(f[2]/AngleStep)+v2, (int)(f[3]/DistanceStep)+v3);
          KeyType hashValue = hashPPF(key);
          neighbors.push_back(hashValue);
        }
      }
    }
  }
  return neighbors;
}

  
PPFDetector::~PPFDetector()
{
  clearTrainedModel();
}

void PPFDetector::trainModel(const std::string &model_path)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sampled(new pcl::PointCloud<pcl::PointNormal>);

  float sampling_step_model = parameters["Downsampling.Step.Train.Model"];
  float scale_x = parameters["Model.Scale.x"];
  float scale_y = parameters["Model.Scale.y"];
  float scale_z = parameters["Model.Scale.z"];
  
  loadSTL(model, model_path.c_str(), 50000, scale_x, scale_y, scale_z);
  downSampling(model,sampled,sampling_step_model);

  Eigen::Vector2f xr,yr,zr;
  computeBboxStd(sampled,xr,yr,zr);

  float dx = xr[1]-xr[0];
  float dy = yr[1]-yr[0];
  float dz = zr[1]-zr[0];
  float diameter = sqrt(dx*dx+dy*dy+dz*dz);
  float distanceStep = (float)(diameter*sampling_step_relative);
  this->model_diameter = diameter;

  float angleStep = angle_step;
  
  int num_points = sampled->points.size();
  int num_ref = num_points;
  int size = num_ref*num_ref;
  
  hash_nodes = (THash*)calloc(size,sizeof(THash));
  
  hash_table.reserve(size);
  
  double lamda = 0.98;
  for(int i=0; i<num_ref; i++)
  {
    if(i%10 == 0)
        std::cout<<"trained "<<i*100/num_ref+1<<"%"<<std::endl;
    pcl::PointNormal p1 = sampled->points[i];
    const Eigen::Vector3f pos1(p1.x,p1.y,p1.z);
    const Eigen::Vector3f nor1(p1.normal[0],p1.normal[1],p1.normal[2]);
    
    for(int j=0; j<num_points; j++)
    {
      if(j != i)
      {
        bool hasNaN = false;
        
        pcl::PointNormal p2 = sampled->points[j];
        const Eigen::Vector3f pos2(p2.x,p2.y,p2.z);
        const Eigen::Vector3f nor2(p2.normal[0],p2.normal[1],p2.normal[2]);
        
        Eigen::Vector4f f(0.0,0.0,0.0,0.0);
        computePPFFeatures(pos1,nor1,pos2,nor2,f);
        
        for(int id=0; id<4; id++)
        {
            if(isnan(f(id)))
                hasNaN = true;
        }
        if(hasNaN)
            continue;
        
        KeyType hashValue = hashPPF(f,angleStep,distanceStep);
        
        double alpha = computeAlpha(pos1,nor1,pos2);
        uint Ind = i*num_points+j;
        
        double dp = nor1[0]*nor2[0]+nor1[1]*nor2[1]+nor1[2]*nor2[2];
        double voteVal = 1-lamda*abs(dp);
        
        THash* hashNode = &hash_nodes[Ind];
        hashNode->i = i;
        hashNode->ppfInd = Ind;
        hashNode->alpha = alpha;
        hashNode->voteVal = voteVal;
        
        if(hash_table.find(hashValue)!= hash_table.end())
            hash_table[hashValue].push_back(hashNode);
        else
            hash_table.emplace(hashValue,std::vector<THash*>{hashNode});
      }
    }
  }
  
  distance_step = distanceStep;
  num_ref_points = num_ref;
  sampled_pc = sampled;
  trained = true;
}


void PPFDetector::setScene(const std::string& empty_scene, const std::string& scene_pc, const std::string& model_path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr object_normal(new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_copy(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr object_inliers(new pcl::PointCloud<pcl::PointNormal>);

    float position_thre = parameters["Remove.Background.Position.Threshold"];
    float color_thre = parameters["Remove.Background.Color.Threshold"];
    float sampling_step_scene = parameters["Downsampling.Step.Set.Scene"];
    float scale_x = parameters["Model.Scale.x"];
    float scale_y = parameters["Model.Scale.y"];
    float scale_z = parameters["Model.Scale.z"];
    int whether_remove_plane = parameters["Whether.Remove.Plane"];
    
    loadPLYRGB(scene, scene_pc.c_str());
    loadPLYRGB(empty, empty_scene.c_str());
    removeBG(scene,empty,scene_object,position_thre,color_thre);

    computeNormals(scene_object,object_normal,sampling_step_scene);

    Eigen::Vector3d bounds = getSize(model_path.c_str(),scale_x,scale_y,scale_z);
    Eigen::Vector3d new_bounds;
    pcl::PointNormal centre_point;

    if(whether_remove_plane)
    {
        removePlane(object_normal,filtered);
        pcl::copyPointCloud(*filtered,*filtered_copy);
        computeBBOX(filtered,bounds,new_bounds,centre_point);
    }
    else
        computeBBOX(object_normal,bounds,new_bounds,centre_point);

    obj_centre = centre_point;

    double diameter1 = std::sqrt(bounds(0)*bounds(0)+bounds(1)*bounds(1)+bounds(2)*bounds(2));
    double diameter2 = std::sqrt(new_bounds(0)*new_bounds(0)+new_bounds(1)*new_bounds(1)+new_bounds(2)*new_bounds(2));
    double search_radius;
    if(diameter1>diameter2)
        search_radius = diameter1/2.0;
    else
        search_radius = diameter2/2.0;

    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    std::vector<int> indices;
    std::vector<float> distances;
    if(whether_remove_plane)
    {
        kdtree.setInputCloud(filtered);
        kdtree.radiusSearch(centre_point,search_radius,indices,distances);
        for(int id:indices)
            object_inliers->push_back(filtered->points[id]);
    }
    else
    {
        kdtree.setInputCloud(object_normal);
        kdtree.radiusSearch(centre_point,search_radius,indices,distances);
        for(int id:indices)
            object_inliers->push_back(object_normal->points[id]);
    }

    sampled_scene = object_inliers;

    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("scene"));
    viewer->addPointCloudNormals<pcl::PointNormal,pcl::PointNormal>(filtered,filtered,1,1,"normals");
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    */
}


//matching
void PPFDetector::match()
{
  if(!trained)
    std::cerr<<"the model haven't been trained"<<endl;

  float minimum_supports = parameters["Match.Minimum.Supports"];

  distance_step = model_diameter*sampling_step_relative;

  int numAngles = (int)angle_step_relative;
  float distanceStep = (float)distance_step;

  uint n = sampled_pc->points.size();
  std::vector<Pose3DPtr> poseList;
  int sceneSamplingStep = this->scene_skip_length;

  pcl::PointCloud<pcl::PointNormal>::Ptr scene_sampled(new pcl::PointCloud<pcl::PointNormal>);
  scene_sampled = this->sampled_scene;
  poseList.reserve(scene_sampled->points.size()/sceneSamplingStep+4);

  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud(scene_sampled);
  std::vector<int> indices;
  std::vector<float> distances;

#if defined _OPENMP
#pragma omp parallel for
#endif
  for(int i=0; i<scene_sampled->points.size(); i += sceneSamplingStep)
  {
    std::cout<<"matching "<<i*100/scene_sampled->points.size()+1<<"%"<<std::endl;
    uint refIndMax = 0, alphaIndMax = 0;
    double maxVotes = 0;
    pcl::PointNormal pt1 = scene_sampled->points[i];

    const Eigen::Vector3f p1(pt1.x,pt1.y,pt1.z);
    const Eigen::Vector3f n1(pt1.normal[0],pt1.normal[1],pt1.normal[2]);
    Eigen::Matrix3d Rsg = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d RInv = Eigen::Matrix3d::Zero();
    Eigen::Vector3d tsg(0.0,0.0,0.0);

    computeTransformRT(p1,n1,Rsg,tsg);

    double* accumulator = (double*)calloc(numAngles*n,sizeof(double));

    indices.clear();
    distances.clear();
    if(kdtree.radiusSearch(pt1,1.2*model_diameter,indices,distances)<0)
     continue;

    for(int j=0; j<indices.size(); j++)
    {
      int index = indices[j];
      if(index!=i)
      {
        pcl::PointNormal pt2 = scene_sampled->points[index];
        const Eigen::Vector3f p2(pt2.x,pt2.y,pt2.z);
        const Eigen::Vector3f n2(pt2.normal[0],pt2.normal[1],pt2.normal[2]);

        Eigen::Vector3d p2t;
        double alpha_scene;

        Eigen::Vector4f f = Eigen::Vector4f::Zero();
        computePPFFeatures(p1,n1,p2,n2,f);

        std::vector<KeyType> hashValues = searchNeighbors(f,angle_step,distanceStep);
        //do not search neighbors
        //KeyType hashValue_ = hashPPF(f,angle_step,distanceStep);

        p2t = tsg+Rsg*p2.cast<double>();
        alpha_scene=atan2(-p2t[2], p2t[1]);
        if(sin(alpha_scene)*p2t[2]>0.0)
          alpha_scene = -alpha_scene;


        for(KeyType hashValue:hashValues)
        {
          if(hash_table.find(hashValue) != hash_table.end())
          {
            std::vector<THash*> nodes = hash_table[hashValue];
            for(auto node:nodes)
            {
                int corrI = (int)node->i;
                int ppfInd = (int)node->ppfInd;
                double alpha_model = (double)node->alpha;
                double voteVal = (double)node->voteVal;
                double alpha = alpha_model-alpha_scene;

                int alpha_index = (int)(numAngles*(alpha+2*M_PI)/(4*M_PI));
                uint accIndex = corrI*numAngles + alpha_index;

                //accumulator[accIndex]++;
                accumulator[accIndex] = accumulator[accIndex]+voteVal;
            }
          }
        }

        /*
        if(hash_table.find(hashValue) != hash_table.end())
        {
          std::vector<THash*> nodes = hash_table[hashValue];
          for(auto node:nodes)
          {
              int corrI = (int)node->i;
              int ppfInd = (int)node->ppfInd;
              double alpha_model = (double)node->alpha;
              double voteVal = (double)node->voteVal;
              //float* ppfCorrScene = ppf.ptr<float>(ppfInd);
              //double alpha_model = (double)ppfCorrScene[PPF_LENGTH-1];
              double alpha = alpha_model-alpha_scene;

              int alpha_index = (int)(numAngles*(alpha+2*M_PI)/(4*M_PI));
              uint accIndex = corrI*numAngles + alpha_index;

              //accumulator[accIndex]++;
              accumulator[accIndex] = accumulator[accIndex]+voteVal;
          }
        }
        */

      }
    }

    //maximize the accumulator
    for (uint k = 0; k < n; k++)
    {
      for (int j = 0; j < numAngles; j++)
      {
        const uint accInd = k*numAngles + j;
        const double accVal = accumulator[accInd];
        if (accVal > maxVotes)
        {
          maxVotes = accVal;
          refIndMax = k;
          alphaIndMax = j;
        }
#if !defined (_OPENMP)
        accumulator[accInd] = 0;
#endif
      }
    }

    if(maxVotes>minimum_supports)
    {
      Eigen::Vector3d tInv,tmg;
      Eigen::Matrix3d Rmg;
      RInv = Rsg.transpose();
      tInv = -RInv*tsg;

      Eigen::Isometry3d TsgInv = Eigen::Isometry3d::Identity();
      TsgInv.matrix().topRightCorner<3,1>() = tInv;
      TsgInv.matrix().block<3,3>(0,0) = RInv;

      pcl::PointNormal ref = sampled_pc->points[refIndMax];

      const Eigen::Vector3f pMax(ref.x,ref.y,ref.z);
      const Eigen::Vector3f nMax(ref.normal_x,ref.normal_y,ref.normal_z);

      computeTransformRT(pMax,nMax,Rmg,tmg);

      Eigen::Isometry3d Tmg = Eigen::Isometry3d::Identity();
      Tmg.matrix().topRightCorner<3,1>() = tmg;
      Tmg.matrix().block<3,3>(0,0) = Rmg;

      //convert alpha_index to alpha
      int alpha_index = alphaIndMax;
      double alpha = (alpha_index*(4*M_PI))/numAngles-2*M_PI;

      Eigen::Isometry3d Talpha = Eigen::Isometry3d::Identity();
      Eigen::Matrix3d R;
      Eigen::Vector3d t(0.0,0.0,0.0);
      getUnitXRotation(alpha,R);

      Talpha.matrix().topRightCorner<3,1>() = t;
      Talpha.matrix().block<3,3>(0,0) =  R;

      Eigen::Isometry3d rawPose = TsgInv*(Talpha*Tmg);

      Pose3DPtr pose(new Pose3D(alpha,refIndMax,maxVotes));
      pose->updatePose(rawPose);
      #if defined (_OPENMP)
      #pragma critical
      #endif
      {
        poseList.push_back(pose);
      }
    }

    free(accumulator);
  }

  std::cout<<"total number of poses "<<poseList.size()<<std::endl;

  std::vector<Pose3DPtr> results;
  int numPosesAdded = poseList.size();
  clusterPoses(poseList,numPosesAdded,results);
  poses = results;

  std::cout<<"number of poses after clustering "<<results.size()<<std::endl;
}

bool PPFDetector::matchPose(const Pose3D& sourcePose, const Pose3D& targetPose)
{
  Eigen::Vector3d v = targetPose.t-sourcePose.t;
  double dNorm = v.norm();
  const double phi = fabs(targetPose.angle-sourcePose.angle);
  return (dNorm<this->position_threshold && phi<this->rotation_threshold);
}


void PPFDetector::clusterPoses(std::vector<Pose3DPtr>& poseList, int numPoses, std::vector<Pose3DPtr>& finalPoses)
{
  std::vector<PoseCluster3DPtr> poseClusters;
  
  finalPoses.clear();

  // sort the poses for stability
  std::sort(poseList.begin(), poseList.end(), sortPoses);

  for (int i=0; i<numPoses; i++)
  {
    Pose3DPtr pose = poseList[i];
    bool assigned = false;

    // search all clusters
    for (size_t j=0; j<poseClusters.size() && !assigned; j++)
    {
      const Pose3DPtr poseCenter = poseClusters[j]->poseList[0];
      if (matchPose(*pose, *poseCenter))
      {
        poseClusters[j]->addPose(pose);
        assigned = true;
      }
    }

    if (!assigned)
    {
      poseClusters.push_back(PoseCluster3DPtr(new PoseCluster3D(pose)));
    }
  }
  
  //sort the clusters
  std::sort(poseClusters.begin(),poseClusters.end(),sortPoseClusters);
  finalPoses.resize(poseClusters.size());
  
  if (use_weighted_avg)
  {
#if defined _OPENMP
#pragma omp parallel for
#endif
    // uses weighting by the number of votes
    for (int i=0; i<static_cast<int>(poseClusters.size()); i++)
    {
      // We could only average the quaternions
      Eigen::Vector4d qAvg = Eigen::Vector4d::Zero();
      Eigen::Vector3d tAvg = Eigen::Vector3d::Zero();

      // Perform the final averaging
      PoseCluster3DPtr curCluster = poseClusters[i];
      std::vector<Pose3DPtr> curPoses = curCluster->poseList;
      int curSize = (int)curPoses.size();
      double numTotalVotes = 0.0;

      for (int j=0; j<curSize; j++)
        numTotalVotes += curPoses[j]->numVotes;

      double wSum=0;

      for (int j=0; j<curSize; j++)
      {
        const double w = (double)curPoses[j]->numVotes / (double)numTotalVotes;

        qAvg += w * curPoses[j]->q.coeffs();
        tAvg += w * curPoses[j]->t;
        wSum += w;
      }

      tAvg *= 1.0 / wSum;
      qAvg *= 1.0 / wSum;

      Eigen::Quaterniond qua(qAvg[3],qAvg[0],qAvg[1],qAvg[2]);
      qua.normalize();
      
      curPoses[0]->updatePoseQuat(qua, tAvg);
      curPoses[0]->numVotes=curCluster->numVotes;

      finalPoses[i]=curPoses[0]->clone();
    }
  }
  else
  {
#if defined _OPENMP
#pragma omp parallel for
#endif
    for (int i=0; i<static_cast<int>(poseClusters.size()); i++)
    {
      // We could only average the quaternions
      Eigen::Vector4d qAvg = Eigen::Vector4d::Zero();
      Eigen::Vector3d tAvg = Eigen::Vector3d::Zero();

      // Perform the final averaging
      PoseCluster3DPtr curCluster = poseClusters[i];
      std::vector<Pose3DPtr> curPoses = curCluster->poseList;
      const int curSize = (int)curPoses.size();

      for (int j=0; j<curSize; j++)
      {
        qAvg += curPoses[j]->q.coeffs();
        tAvg += curPoses[j]->t;
      }

      tAvg *= 1.0 / curSize;
      qAvg *= 1.0 / curSize;

      Eigen::Quaterniond qua(qAvg[3],qAvg[0],qAvg[1],qAvg[2]);
      qua.normalize();
      
      curPoses[0]->updatePoseQuat(qua, tAvg);
      curPoses[0]->numVotes=curCluster->numVotes;

      finalPoses[i]=curPoses[0]->clone();
    }
  }
  poseClusters.clear();
}

void PPFDetector::rescore()
{
    std::cout<<"rescore ......"<<std::endl;

    float rescoring_threshold = parameters["Rescore.Threshold"];
    
    std::vector<std::pair<int,Pose3DPtr>> score_pose;
    
    std::vector<Pose3DPtr> vposes;
    if(poses.size() > 100)
        vposes = std::vector<Pose3DPtr>(poses.begin(),poses.begin()+100);
    else
        vposes = poses;

    /*
    for(auto&  p:vposes)
    {
        p->pose.matrix().topRightCorner<3,1>() = Eigen::Vector3d(obj_centre.x,obj_centre.y,obj_centre.z);
    }
    */
    pcl::PointCloud<pcl::PointNormal>::Ptr scene(sampled_scene);
    pcl::PointCloud<pcl::PointNormal>::Ptr model(sampled_pc);
    
    std::vector<Pose3DPtr> scored_poses;
    
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(scene);
    std::vector<int> indices;
    std::vector<float> distances;
    
    for(int i=0; i<vposes.size(); i++)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointNormal>);
        
        int num = 0;
        Eigen::Matrix4d T = vposes[i]->pose.matrix();
        pcl::transformPointCloud(*model,*transformed,T);
        for(int j=0; j<transformed->points.size(); j++)
        {
            indices.clear();
            distances.clear();
            kdtree.nearestKSearch(transformed->points[j],1,indices,distances);
            if(distances[0]<rescoring_threshold)
                num++;
        }
        
        vposes[i]->numVotes = num;
        score_pose.push_back(std::make_pair(num,vposes[i]));
        
        //std::cout<<"pose "<<i<<" votes: "<<num<<std::endl;
        
    }
    
    std::sort(score_pose.begin(),score_pose.end(),sortRescore);
    
    for(int i=0; i<score_pose.size(); i++)
    {
        scored_poses.push_back(score_pose[i].second);
    }

    poses = scored_poses;
    
}

void PPFDetector::rescoreAfterICP()
{
    std::cout<<"rescore after icp ......"<<std::endl;
    float threshold = parameters["Rescore.ICP.Threshold"];

    pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr temp(new pcl::PointCloud<pcl::PointNormal>);
    
    std::vector<std::pair<int,Pose3DPtr>> score_pose;
    
    std::vector<Pose3DPtr> vposes;
    if(poses.size() > 10)
        vposes = std::vector<Pose3DPtr>(poses.begin(),poses.begin()+10);
    else
        vposes = poses;
    
    pcl::copyPointCloud(*sampled_scene,*scene);
    pcl::IterativeClosestPoint<pcl::PointNormal,pcl::PointNormal> icp;
    
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(scene);
    std::vector<int> indices;
    std::vector<float> distances;
    
    std::vector<Pose3DPtr> scored_poses;
    for(int i=0; i<vposes.size(); i++)
    {
        Pose3DPtr p = vposes[i];
        Eigen::Matrix4d ini_T = p->pose.matrix();
        
        pcl::copyPointCloud(*sampled_pc,*model);
        transformPointCloud(model,ini_T);
        
        icp.setInputSource(model);
        icp.setInputTarget(scene);
        icp.setTransformationEpsilon(1e-10);
        icp.setMaxCorrespondenceDistance(threshold);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setMaximumIterations(35);
        
        icp.align(*temp);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
        
        int num = 0;
        for(int j=0; j<temp->points.size(); j++)
        {
            indices.clear();
            distances.clear();
            kdtree.nearestKSearch(temp->points[j],1,indices,distances);
            if(distances[0]<threshold)
                num++;
        }
        
        vposes[i]->numVotes = num;
        //Eigen::Isometry3d append_pose;
        //append_pose.matrix() = T;
        //vposes[i]->appendPose(append_pose);
        score_pose.push_back(std::make_pair(num,vposes[i]));
    }
    
    std::sort(score_pose.begin(),score_pose.end(),sortRescore);
    
    for(int i=0; i<score_pose.size(); i++)
    {
        scored_poses.push_back(score_pose[i].second);
    }

    poses = scored_poses;
}

Pose3DPtr PPFDetector::getPoseAfterICP()
{
    std::cout<<"final icp ......"<<std::endl;
    float icp_thre = parameters["ICP.Threshold"];

    pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene(new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr temp1(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr temp2(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr final(new pcl::PointCloud<pcl::PointNormal>);
    
    std::vector<Pose3DPtr> poses_ini = poses;
    
    std::vector<Pose3DPtr> vposes;
    if(poses_ini.size() > 5)
        vposes = std::vector<Pose3DPtr>(poses_ini.begin(),poses_ini.begin()+5);
    else
        vposes = poses_ini;
    
    int max_votes=0;
    Pose3DPtr best_pose;
    
    pcl::copyPointCloud(*sampled_scene,*scene);
    pcl::IterativeClosestPoint<pcl::PointNormal,pcl::PointNormal> icp;
    
    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(scene);
    std::vector<int> indices;
    std::vector<float> distances;
    
    for(auto pose:vposes)
    {
        pcl::copyPointCloud(*sampled_pc,*model);
    
        Eigen::Matrix4d init_T = pose->pose.matrix();
        transformPointCloud(model,init_T);
        
        icp.setInputSource(model);
        icp.setInputTarget(scene);
        icp.setTransformationEpsilon(1e-10);
        icp.setMaxCorrespondenceDistance(icp_thre);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setMaximumIterations(35);

        icp.align(*temp1);

        Eigen::Matrix4d T1 = icp.getFinalTransformation().cast<double>();
        
        //
        icp.setInputSource(temp1);
        icp.setInputTarget(scene);
        icp.setTransformationEpsilon(1e-10);
        icp.setMaxCorrespondenceDistance(0.6*icp_thre);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setMaximumIterations(35);

        icp.align(*temp2);

        Eigen::Matrix4d T2 = icp.getFinalTransformation().cast<double>();
        
        icp.setInputSource(temp2);
        icp.setInputTarget(scene);
        icp.setTransformationEpsilon(1e-10);
        icp.setMaxCorrespondenceDistance(0.6*0.6*icp_thre);
        icp.setEuclideanFitnessEpsilon(0.001);
        icp.setMaximumIterations(35);
        
        icp.align(*final);
        
        Eigen::Matrix4d T3 = icp.getFinalTransformation().cast<double>();
        
        int num = 0;
        for(int j=0; j<temp2->points.size(); j++)
        {
            indices.clear();
            distances.clear();
            kdtree.nearestKSearch(temp2->points[j],1,indices,distances);
            if(distances[0]<0.6*0.6*icp_thre)
                num++;
        }
        
        //std::cout<<num<<std::endl;
        
        if(num>max_votes)
        {
            max_votes = num;
            
            Eigen::Isometry3d append_pose;
            append_pose.matrix() = T1*T2*T3;
            pose->appendPose(append_pose);
            best_pose = pose;
            
            //std::cout<<best_pose->pose.matrix()<<std::endl;
        }
        
    }
    std::cout<<best_pose->pose.matrix()<<std::endl;

    return best_pose;
}

void PPFDetector::save_model(const char* file)
{
    std::cout<<"saving model ......"<<std::endl;

    std::ofstream of(file);

    of<<"model parameters"<<endl;
    of<<"model diameter "<<model_diameter<<endl;
    of<<"hash table size "<<hash_table.size()<<endl;
    of<<"point cloud size "<<sampled_pc->points.size()<<endl;
    of<<"end header"<<endl;

    for(const auto &item:hash_table)
    {
        of<<item.first<<" "<<item.second.size()<<" ";
        for(const auto node:item.second)
        {
            of<<node->ppfInd<<" "<<node->i<<" "<<node->alpha<<" "<<node->voteVal<<" ";
        }
        of<<endl;
    }

    for(int i=0; i<sampled_pc->points.size(); i++)
    {
        of<<sampled_pc->points[i].x<<" "<<sampled_pc->points[i].y<<" "<<sampled_pc->points[i].z<<" "<<sampled_pc->points[i].normal_x<<" "<<sampled_pc->points[i].normal_y<<" "<<sampled_pc->points[i].normal_z<<endl;
    }

    of.close();
}

void PPFDetector::load_model(const char* file)
{
    std::cout<<"loading model ......"<<std::endl;

    std::ifstream in(file);

    std::string tbSZ = "hash table size ";
    std::string pcSZ = "point cloud size ";
    std::string diameter_flag = "model diameter ";
    int p1 = diameter_flag.size();
    int p2 = tbSZ.size();
    int p3 = pcSZ.size();

    int n1,n2,ppf_rows;

    std::string line;
    while(!in.eof())
    {
        getline(in,line);
        if(line.find(diameter_flag)!=std::string::npos)
        {
            std::string num = line.substr(p1);
            model_diameter = atof(num.c_str());
        }
        if(line.find(tbSZ)!=std::string::npos)
        {
            std::string num = line.substr(p2);
            n1 = atoi(num.c_str());
        }
        if(line.find(pcSZ)!= std::string::npos)
        {
            std::string num = line.substr(p3);
            n2 = atoi(num.c_str());
        }

        if(line=="end header")
        {
            break;
        }
    }

    std::unordered_map<KeyType,std::vector<THash*>> table;
    table.reserve(n1);
    std::vector<THash*> value;

    KeyType key;
    int nums,i,ppfInd;
    double alpha,voteVal;

    while(!in.eof() && n1>0)
    {
        in>>key>>nums;
        value.clear();
        while(nums>0)
        {
            in>>ppfInd>>i>>alpha>>voteVal;

            THash* node(new THash);

            node->ppfInd = ppfInd;
            node->i = i;
            node->alpha = alpha;
            node->voteVal = voteVal;
            value.push_back(node);
            nums--;
        }

        table.emplace(key,value);
        getline(in,line);
        n1--;
    }

    hash_table = table;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    cloud->resize(n2);
    float x,y,z,nx,ny,nz;
    int n=0;
    while(!in.eof() && n<n2)
    {
        in>>x>>y>>z>>nx>>ny>>nz;
        if(in.fail())
            break;
        cloud->points[n].x = x;
        cloud->points[n].y = y;
        cloud->points[n].z = z;
        cloud->points[n].normal_x = nx;
        cloud->points[n].normal_y = ny;
        cloud->points[n].normal_z = nz;

        getline(in,line);
        n++;
    }

    sampled_pc = cloud;

    trained = true;

    in.close();
}

void PPFDetector::save_poses(const char* file)
{
    std::ofstream of(file);

    for(int i=0; i<poses.size(); i++)
    {
        of<<poses[i]->pose.matrix()<<endl;
    }

    of.close();
}

void PPFDetector::load_poses(const char* file)
{
    std::ifstream in(file);

    std::vector<double> vec;
    double n1,n2,n3,n4;
    std::string line;

    std::vector<Pose3DPtr> poses_loaded;
    Eigen::Matrix4d pose;
    int i=0;
    while(!in.eof())
    {
        in>>n1>>n2>>n3>>n4;
        pose.row(i)<<n1,n2,n3,n4;
        i++;
        getline(in,line);
        if(i>3)
        {
            i=0;
            Pose3DPtr p;
            p->pose.matrix() = pose;
            poses_loaded.push_back(p);
        }
    }
    in.close();

    poses = poses_loaded;
}

/***********************************************************************************/
//this function is for finding the most similar gear
int PPFDetector::getScore(Pose3DPtr pose, const char* model, const char* scene, const float threshold)
{
    std::cout<<"calculate score ......"<<std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr model_sampled(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_sampled(new pcl::PointCloud<pcl::PointNormal>);
    loadSTL(model_cloud,model,50000,10,10,10);
    loadSTL(scene_cloud,scene,50000,1,1,1);
    downSampling(model_cloud,model_sampled,0.5);
    downSampling(scene_cloud,scene_sampled,0.5);

    pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(scene_sampled);
    std::vector<int> indices;
    std::vector<float> distances;

    pcl::PointCloud<pcl::PointNormal>::Ptr transformed(new pcl::PointCloud<pcl::PointNormal>);

    int num = 0;
    Eigen::Matrix4d T = pose->pose.matrix();
    pcl::transformPointCloud(*model_sampled,*transformed,T);
    for(int j=0; j<transformed->points.size(); j++)
    {
        indices.clear();
        distances.clear();
        kdtree.nearestKSearch(transformed->points[j],1,indices,distances);
        if(distances[0]<threshold)
            num++;
    }

    return num;
}
       

void PPFDetector::test()
{
}
    
}
