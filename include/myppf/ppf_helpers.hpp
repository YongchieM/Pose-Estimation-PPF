#ifndef __MYPPF_HELPERS_HPP__
#define __MYPPF_HELPERS_HPP__

#include<string>

#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<pcl/common/common.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/filter.h>
#include<pcl/filters/uniform_sampling.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/normal_space.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/surface/mls.h>

#include<pcl/ModelCoefficients.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_clusters.h>

#include<pcl/features/moment_of_inertia_estimation.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/features/integral_image_normal.h>

#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/console/print.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/obj_io.h>
#include<pcl/io/vtk_lib_io.h>
#include<pcl/PolygonMesh.h>

#include <vtkVersion.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkLODActor.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkMassProperties.h>

#include<pcl/kdtree/kdtree_flann.h>
#include<boost/filesystem.hpp>
#include<boost/thread/thread.hpp>
#include<fstream>
#include<flann/flann.h>

namespace myppf
{
void loadPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, const char* filename);
void loadPLYRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PC, const char* filename);

//load CAD model, sample points on the model and compute normals
void loadOBJ(pcl::PointCloud<pcl::PointNormal>::Ptr PC, const char* filename, const int samples, const bool flipViewpoint=false);
void loadSTL(pcl::PointCloud<pcl::PointNormal>::Ptr PC, const char* filename, const int samples, const int scale_x, const int scale_y, const int scale_z, const bool flipViewpoint=false);
inline double uniform_deviate(int seed);
inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector3f& p);
inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3f& p, bool calcNormal, Eigen::Vector3f& n);
void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal>& cloud_out, const bool flipViewpoint);

//compute the bounding-box of given point cloud
void computeBboxStd(pcl::PointCloud<pcl::PointNormal>::Ptr PC, Eigen::Vector2f& xrange, Eigen::Vector2f& yrange, Eigen::Vector2f& zrange);

//down-sampling method in the paper
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, Eigen::Vector2f& xrange, Eigen::Vector2f& yrange, Eigen::Vector2f& zrange, const float sample_step_relative, const double threshold,const int num_threshold);
void addToClusters(std::vector<std::vector<pcl::PointNormal>>& clusters, pcl::PointNormal point, const double threshold);
void group(std::vector<pcl::PointNormal>& groups, pcl::PointCloud<pcl::PointNormal>::Ptr PC, std::vector<int>& points, const double threshold);
void average(std::vector<pcl::PointNormal>& new_pc, std::vector<int>& points, std::vector<pcl::PointNormal>& sub_pc, const double threshold, const int num_threshold);

//simple down-sampling
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const float leaf_size);
void downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointXYZ>::Ptr SampledPC, const float leaf_size);
//down-sampling with different step size along z-axis
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const float leaf_size, const float z_ratio);
//down-sampling in normal space, similar with the method in the paper
void downSamplingNormals(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const int num=2500);

//normal estimation method based on PCA
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, float leaf_size, const bool FlipViewpoint=false);
//normal estimation method in the paper, fast but not good
void computeNormals_on(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, float leaf_size);

//remove the background
void removeBG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold, const double colorThreshold);
void removeBG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold);
void removeBGWithPatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold, const double colorThreshold, const int patch_size=7);
bool corr(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, const double positionThreshold, const double colorThreshold);
bool corr(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, const double positionThreshold);
bool searchInPatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, int i, int j, const int patch_size, const double positionThreshold, const double colorThreshold);

//transform the point cloud with a pose matrix
void transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Matrix4d& T);
//compute the size of a CAD model
Eigen::Vector3d getSize(const char *filename, const float scale_x, const float scale_y, const float scale_z);

//estimate plane and remove plane
void removePlane(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr filtered);
//find the oriented bounding-box of object in the scene
void computeBBOX(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Vector3d& bounds, Eigen::Vector3d& box_size, pcl::PointNormal& obj_centre);
void sortEigenVector(Eigen::Vector3d &vec);
/**********************************************************/
//functions for finding the most similar gear, not important for this prooject
int getFileNames(const std::string& dir, std::vector<std::string>& filenames);
std::vector<std::string> processSTL(const char* filename, const std::string& dir);
int getTeeth(cv::Mat& gear_im);
int getTeeth(const char* filename);
double getSize(const char* filename);
double getVolume(const char* filename, const float scale, const float leaf_size, double& area);
void samplingOnModel(const char* filename, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, const int samples, const float leaf_size);
void samplingOnModel(const char* filename, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, Eigen::Vector3d& bounds, const float leaf_size);
//
cv::Mat modelToImg(const char* filename);

}

#endif
