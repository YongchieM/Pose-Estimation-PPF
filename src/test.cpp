#include<iostream>
#include<opencv2/opencv.hpp>

#include "../include/myppf/headers.hpp"

using namespace std;
using namespace myppf;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_model(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_scene(new pcl::PointCloud<pcl::PointXYZRGB>);

    string empty_scene = "/open_source_project/6d-pose-estimation-using-ppf/data/empty_scene.ply";
    string full_scene = "/open_source_project/6d-pose-estimation-using-ppf/data/scene.ply";
    string model_path = "/open_source_project/6d-pose-estimation-using-ppf/data/siemens/siemens_gear2_1.stl";

    //string model_path = argv[1];
    //string full_scene = argv[2];
    //string empty_scene = argv[3];


    string strSettingPath = "../para/settings.yaml";

    //for visualization
    loadPLYRGB(colored_scene,full_scene.c_str());
    loadSTL(PCNormals, model_path.c_str(), 50000, 15, 15, 20);
    colored_model->points.resize(PCNormals->points.size());
    for(int i=0; i<PCNormals->points.size(); i++)
    {
        colored_model->points[i].x = PCNormals->points[i].x;
        colored_model->points[i].y = PCNormals->points[i].y;
        colored_model->points[i].z = PCNormals->points[i].z;
        colored_model->points[i].r = 0;
        colored_model->points[i].g = 255;
        colored_model->points[i].b = 0;
    }

    PPFDetector detector(strSettingPath);
    detector.trainModel(model_path);
    //detector.save_model("model.txt");
    //detector.load_model("model.txt");
    detector.setScene(empty_scene,full_scene,model_path);

    detector.match();
    detector.rescore();
    //perform icp for the good poses and rescore
    detector.rescoreAfterICP();
    Pose3DPtr p = detector.getPoseAfterICP();
    Eigen::Matrix4d Tfinal = p->pose.matrix();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*colored_model,*transformed,Tfinal);
    for(int i=0; i<transformed->points.size(); i++)
    {
        colored_scene->points.push_back(transformed->points[i]);
    }


    //point cloud viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_scene,"matching");
    
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
