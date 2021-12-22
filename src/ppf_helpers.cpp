#include "../include/myppf/headers.hpp"

namespace myppf
{
void loadPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, const char* filename)
{
    ifstream inFile(filename, ios::in | ios::binary);
    if (!inFile)
    {
        cerr << "can not open the file" << endl;
        return;
    }

    int numPts;
    std::string line;
    std::string num_flag = "element vertex ";
    int pos = num_flag.size();

    while (!inFile.eof())
    {
        getline(inFile, line);

        if (line.find(num_flag) != std::string::npos)
        {
            std::string num = line.substr(pos);
            numPts = atoi(num.c_str());
            //cout << "There are " << numPts << " points" << endl;
        }

        if (line == "end_header")
            break;
    }

    float position[3];
    unsigned char color[3];

    PC->width = 1920;
    PC->height = 1200;
    PC->resize(PC->width*PC->height);
    PC->is_dense = false;
    pcl::PointXYZ point;

    int n = 0;
    
    while(n < numPts)
    {
        inFile.read((char*)&position, sizeof(position));
        if(!isinf(position[2]) && !isnan(position[2]))
        {
            point.x = position[0];
            point.y = position[1];
            point.z = position[2];
        }
        else
        {
            point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        }
        PC->points[n] = point;
        inFile.ignore(sizeof(color));
        n++;
    }
    
    inFile.close();
}

void loadPLYRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PC, const char* filename)
{
    ifstream inFile(filename, ios::in | ios::binary);
    if (!inFile)
    {
        cerr << "can not open the file" << endl;
        return;
    }

    int numPts;
    std::string line;
    std::string num_flag = "element vertex ";
    int pos = num_flag.size();

    while (!inFile.eof())
    {
        getline(inFile, line);

        if (line.find(num_flag) != std::string::npos)
        {
            std::string num = line.substr(pos);
            numPts = atoi(num.c_str());
            //cout << "There are " << numPts << " points" << endl;
        }

        if (line == "end_header")
            break;
    }

    float position[3];
    unsigned char color[3];

    PC->width = 1920;
    PC->height = 1200;
    PC->resize(PC->width*PC->height);
    PC->is_dense = false;
    pcl::PointXYZRGB point;

    int n = 0;
    while(n<numPts)
    {
        inFile.read((char*)&position, sizeof(position));
        inFile.read((char*)&color, sizeof(color));
        if(!isinf(position[2]) && !isnan(position[2]))
        {
            point.x = position[0];
            point.y = position[1];
            point.z = position[2];
            std::uint8_t r = color[0], g = color[1], b = color[2];    
            std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            point.rgb = *reinterpret_cast<float*>(&rgb);
        }
        else
        {
            point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        }
        PC->points[n] = point;
        n++;
    }
    inFile.close();
}

//load CAD model, sample points on the model and compute normals
void loadOBJ(pcl::PointCloud<pcl::PointNormal>::Ptr PC, const char* filename, const int samples, const bool flipViewpoint)
{
    int SAMPLE_POINTS_ = samples;
    bool with_normals = true;

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    //
    vtkSmartPointer<vtkTransform> pTransform = vtkSmartPointer<vtkTransform>::New();
    pTransform->Scale(1,1,1);
    vtkSmartPointer<vtkTransformPolyDataFilter> pTransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    pTransformPolyDataFilter->SetInputData(polydata);
    pTransformPolyDataFilter->SetTransform(pTransform);
    pTransformPolyDataFilter->Update();

    polydata = pTransformPolyDataFilter->GetOutput();

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
    triangleFilter->SetInput(plydata);
#else
    triangleFilter->SetInputData(polydata);
#endif


    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata = triangleMapper->GetInput();

    uniform_sampling(polydata,SAMPLE_POINTS_,with_normals,*PC, flipViewpoint);
}

void loadSTL(pcl::PointCloud<pcl::PointNormal>::Ptr PC, const char* filename, const int samples, const int scale_x, const int scale_y, const int scale_z, const bool flipViewpoint)
{
    int SAMPLE_POINTS_ = samples;
    bool with_normals = true;

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    //
    vtkSmartPointer<vtkTransform> pTransform = vtkSmartPointer<vtkTransform>::New();
    pTransform->Scale(scale_x,scale_y,scale_z);
    vtkSmartPointer<vtkTransformPolyDataFilter> pTransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    pTransformPolyDataFilter->SetInputData(polydata);
    pTransformPolyDataFilter->SetTransform(pTransform);
    pTransformPolyDataFilter->Update();

    polydata = pTransformPolyDataFilter->GetOutput();

    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
    triangleFilter->SetInput(plydata);
#else
    triangleFilter->SetInputData(polydata);
#endif


    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata = triangleMapper->GetInput();

    uniform_sampling(polydata,SAMPLE_POINTS_,with_normals,*PC, flipViewpoint);
}

inline double uniform_deviate (int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}
 
inline void randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector3f& p)
{
    float r1 = static_cast<float> (uniform_deviate (rand ()));
    float r2 = static_cast<float> (uniform_deviate (rand ()));
    float r1sqr = std::sqrt (r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
}
 
inline void randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector3f& p, bool calcNormal, Eigen::Vector3f& n)
{
    float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);
 
    std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
    vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());
 
    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    polydata->GetCellPoints (el, npts, ptIds);
    polydata->GetPoint (ptIds[0], A);
    polydata->GetPoint (ptIds[1], B);
    polydata->GetPoint (ptIds[2], C);
    if (calcNormal)
    {
        // OBJ: Vertices are stored in a counter-clockwise order by default
        Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
        n = v1.cross (v2);
        n.normalize ();
    }
    randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                         float (B[0]), float (B[1]), float (B[2]),
                         float (C[0]), float (C[1]), float (C[2]), p);
}
 
void uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal>& cloud_out, const bool flipViewpoint)
{
    polydata->BuildCells ();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
 
    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
    {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }
 
    cloud_out.points.resize (n_samples);
    cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
    cloud_out.height = 1;
 
    Eigen::Vector3d viewpoint(500.0,500.0,500.0);
    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector3f p;
        Eigen::Vector3f n;
        randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
        Eigen::Vector3d point(p[0],p[1],p[2]);
        Eigen::Vector3d normal(n[0],n[1],n[2]);
        if(flipViewpoint)
            flipNormalViewpoint(point,viewpoint,normal);
        if (calc_normal)
        {
            cloud_out.points[i].normal_x = normal[0];
            cloud_out.points[i].normal_y = normal[1];
            cloud_out.points[i].normal_z = normal[2];
        }
    }
}

//down-sampling method in the paper, didn't use it
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, Eigen::Vector2f& xrange, Eigen::Vector2f& yrange, Eigen::Vector2f& zrange, const float sample_step_relative, const double threshold,const int num_threshold)
{
    std::vector<std::vector<int>> map;
    std::vector<std::vector<int>> new_map;
    std::vector<pcl::PointNormal> groups;
    std::vector<pcl::PointNormal> new_pc;
    std::vector<pcl::PointNormal> sub_pc;
    SampledPC->points.reserve(PC->points.size());

    int numSamplesDim = (int)(1.0/sample_step_relative);

    float x_r = xrange[1]-xrange[0];
    float y_r = yrange[1]-yrange[0];
    float z_r = zrange[1]-zrange[0];

    map.resize((numSamplesDim+1)*(numSamplesDim+1)*(numSamplesDim+1));

    for(int i=0; i<PC->points.size();)
    {
        int xCell = (int)((float)numSamplesDim*(PC->points[i].x-xrange[0])/x_r);
        int yCell = (int)((float)numSamplesDim*(PC->points[i].y-yrange[0])/y_r);
        int zCell = (int)((float)numSamplesDim*(PC->points[i].z-zrange[0])/z_r);
        int index = xCell*numSamplesDim*numSamplesDim+yCell*numSamplesDim+zCell;
        map[index].push_back(i);
        i+=2;
    }

    for(int i=0; i<map.size(); i++)
    {
        if(!map[i].empty())
        {
            group(groups,PC,map[i],threshold);
            new_pc.insert(new_pc.end(),groups.begin(),groups.end());
        }
    }

    int ND = (int)(1.0/(sample_step_relative*2.0));
    new_map.resize((ND+1)*(ND+1)*(ND+1));

    for(int i=0; i<new_pc.size();i++)
    {
        int xCell = (int)((float)ND*(new_pc[i].x-xrange[0])/x_r);
        int yCell = (int)((float)ND*(new_pc[i].y-yrange[0])/y_r);
        int zCell = (int)((float)ND*(new_pc[i].z-zrange[0])/z_r);
        int index = xCell*ND*ND+yCell*ND+zCell;
        new_map[index].push_back(i);
    }

    for(int i=0; i<new_map.size(); i++)
    {
        if(!new_map[i].empty())
        {
            average(new_pc,new_map[i],sub_pc,threshold,num_threshold);
            for(int j=0; j<sub_pc.size(); j++)
            {
                SampledPC->push_back(sub_pc[j]);
            }
        }
    }
}

void group(std::vector<pcl::PointNormal>& groups, pcl::PointCloud<pcl::PointNormal>::Ptr PC, std::vector<int>& points, const double threshold)
{
    std::vector<std::vector<pcl::PointNormal>> clusters;
    clusters.reserve(points.size());
    groups.clear(); 
    Eigen::Matrix<float,3,1> average_normal(0.0,0.0,0.0);
    
    for(int index:points)
        addToClusters(clusters, PC->points[index], threshold);
    
    int cluster_num = clusters.size();
    groups.resize(cluster_num);
    
    int n = 0;
    for(int i=0; i<cluster_num; i++)
    {       
        n = clusters[i].size();
        average_normal = Eigen::Vector3f(clusters[i][0].normal_x,clusters[i][0].normal_y,clusters[i][0].normal_z);
        TNormalize3(average_normal);
        
        groups[i].x = clusters[i][0].x/(n-1);
        groups[i].y = clusters[i][0].y/(n-1);
        groups[i].z = clusters[i][0].z/(n-1);
        groups[i].normal_x = average_normal[0];
        groups[i].normal_y = average_normal[1];
        groups[i].normal_z = average_normal[2];
    }
}

void addToClusters(std::vector<std::vector<pcl::PointNormal>>& clusters, pcl::PointNormal point, const double threshold)
{
    Eigen::Vector3f average_normal(0.0,0.0,0.0);
    Eigen::Vector3f point_normal(point.normal_x,point.normal_y,point.normal_z);
    if(clusters.empty())
    {
        clusters.push_back({point, point});
        return;
    }
    else
    {
        for(int i=0; i<clusters.size(); i++)
        {          
            average_normal = Eigen::Vector3f(clusters[i][0].normal_x,clusters[i][0].normal_y,clusters[i][0].normal_z);
            TNormalize3(average_normal);
            
            if(TAngle3Normalized(point_normal, average_normal)<threshold)
            {
                clusters[i].push_back(point);
                
                clusters[i][0].x += point.x;
                clusters[i][0].y += point.y;
                clusters[i][0].z += point.z;
                
                clusters[i][0].normal_x += point.normal_x;
                clusters[i][0].normal_y += point.normal_y;
                clusters[i][0].normal_z += point.normal_z;
    
                return;
            }
        }
        clusters.push_back({point,point});
        return;
    }
} 

void average(std::vector<pcl::PointNormal>& new_pc, std::vector<int>& points, std::vector<pcl::PointNormal>& sub_pc, const double threshold, const int num_threshold)
{
    std::vector<std::vector<pcl::PointNormal>> clusters;
    clusters.reserve(points.size());
    float average_x=0.0, average_y=0.0, average_z=0.0;
    float average_nx=0.0, average_ny=0.0, average_nz=0.0;
    for(int index:points)
    {
        addToClusters(clusters,new_pc[index],threshold);
    }
    
    sub_pc.clear();
    sub_pc.reserve(points.size());
    int irrelevant=0;
    for(int i=0; i<clusters.size(); i++)
    {
        if(clusters[i].size()-1>num_threshold)
        {
            average_x += clusters[i][0].x;
            average_y += clusters[i][0].y;
            average_z += clusters[i][0].z;

            average_nx += clusters[i][0].normal_x;
            average_ny += clusters[i][0].normal_y;
            average_nz += clusters[i][0].normal_z;
            
            irrelevant += clusters[i].size()-1;
            
        }
        else
        {
            for(int j=1; j<clusters[i].size(); j++)
            {
                sub_pc.push_back(clusters[i][j]);
            }
        }
    }
    if(irrelevant==0)
        return;
    
    pcl::PointNormal new_point;
    new_point.x = average_x/irrelevant;
    new_point.y = average_y/irrelevant;
    new_point.z = average_z/irrelevant;
    Eigen::Vector3f average_normal(average_nx,average_ny,average_nz);
    TNormalize3(average_normal);
    new_point.normal_x = average_normal[0];
    new_point.normal_y = average_normal[1];
    new_point.normal_z = average_normal[2];
    sub_pc.push_back(new_point);
}
//compute the bounding-box of given point cloud
void computeBboxStd(pcl::PointCloud<pcl::PointNormal>::Ptr PC, Eigen::Vector2f& xrange, Eigen::Vector2f& yrange, Eigen::Vector2f& zrange)
{
    pcl::PointNormal min, max;
    pcl::getMinMax3D(*PC,min,max);
    xrange[0] = min.x;
    xrange[1] = max.x;
    yrange[0] = min.y;
    yrange[1] = max.y;
    zrange[0] = min.z;
    zrange[1] = max.z;
}

//simple down-sampling
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const float leaf_size)
{
    pcl::VoxelGrid<pcl::PointNormal> vox;
    vox.setInputCloud(PC);
    vox.setLeafSize(leaf_size,leaf_size,leaf_size);
    vox.filter(*SampledPC);
    /*pcl::UniformSampling<pcl::PointNormal> us;
    us.setInputCloud(PC);
    us.setRadiusSearch(leaf_size);
    us.filter(*SampledPC);*/
}

void downSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointXYZ>::Ptr SampledPC, const float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(PC);
    vox.setLeafSize(leaf_size,leaf_size,leaf_size);
    vox.filter(*SampledPC);
    /*pcl::UniformSampling<pcl::PointXYZ> us;
    us.setInputCloud(PC);
    us.setRadiusSearch(leaf_size);
    us.filter(*SampledPC);*/
}
//down-sampling with different step size along z-axis
void downSampling(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const float leaf_size, const float z_ratio)
{
    pcl::VoxelGrid<pcl::PointNormal> vox;
    vox.setInputCloud(PC);
    vox.setLeafSize(leaf_size,leaf_size,leaf_size/z_ratio);
    vox.filter(*SampledPC);
}

//down-sampling in normal space, similar with the method in the paper
void downSamplingNormals(pcl::PointCloud<pcl::PointNormal>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr SampledPC, const int num)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*PC,*normals);
    
    pcl::NormalSpaceSampling<pcl::PointNormal,pcl::PointNormal> nss;
    nss.setInputCloud(PC);
    nss.setNormals(PC);
    nss.setBins(2,2,2);
    nss.setSeed(0);
    nss.setSample(num);
    nss.filter(*SampledPC);
}


//compute normals using PCA
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, float leaf_size, bool FlipViewpoint)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr OutlierRemovedPC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr SampledPC(new pcl::PointCloud<pcl::PointXYZ>);

    downSampling(PC,SampledPC,leaf_size);
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(SampledPC);
    outrem.setRadiusSearch(50);
    outrem.setMinNeighborsInRadius(50);
    outrem.filter(*SampledPC);
    
    //outrem.setInputCloud(SampledPC);
    //outrem.setRadiusSearch(search_radius);
    //outrem.setMinNeighborsInRadius(200);
    //outrem.filter(*SampledPC);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(SampledPC);
    sor.setMeanK(5);
    sor.setStddevMulThresh(1.0);
    sor.filter(*SampledPC);


    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(15);
    n.setInputCloud(SampledPC);
    if(FlipViewpoint)
        n.setViewPoint(0.0,0.0,0.0);
    n.setSearchMethod(tree);
    n.setKSearch(5);
    n.compute(*normals);
    pcl::concatenateFields(*SampledPC,*normals,*PCNormals);
    
    /*
    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
    mls.setInputCloud(SampledPC);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setSearchRadius(5);
    //mls.setNumberOfThreads(3);
    mls.process(*PCNormals);*/
    
}

//normal estimation method in the paper
void computeNormals_on(pcl::PointCloud<pcl::PointXYZ>::Ptr PC, pcl::PointCloud<pcl::PointNormal>::Ptr Out, float leaf_size)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr pcn(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ,pcl::Normal> ne;

    for(int i=0; i<PC->size(); i++)
    {
        PC->points[i].x = (float)(PC->points[i].x/1000.0);
        PC->points[i].y = (float)(PC->points[i].y/1000.0);
        PC->points[i].z = (float)(PC->points[i].z/1000.0);
    }

    ne.setInputCloud(PC);
    
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    
    ne.compute(*normals);
    pcl::concatenateFields(*PC,*normals,*pcn);
    
    bool pInf=false,pNan=false,nInf=false,nNan=false;
    //pcl::PointNormal point;
    //Eigen::Vector3d vp(0.0,0.0,0.0);
    for(int i=0; i<pcn->points.size(); i++)
    {
        pInf = false;
        pNan = false;
        nInf = false;
        nNan = false;
        
        if(isinf(pcn->points[i].x) || isinf(pcn->points[i].y) || isinf(pcn->points[i].z))
            pInf = true;
        if(isnan(pcn->points[i].x) || isnan(pcn->points[i].y) || isnan(pcn->points[i].z))
            pNan = true;
        if(isinf(pcn->points[i].normal[0]) || isinf(pcn->points[i].normal[1]) || isinf(pcn->points[i].normal[2]))
            nInf = true;
        if(isnan(pcn->points[i].normal[0]) || isnan(pcn->points[i].normal[1]) || isnan(pcn->points[i].normal[2]))
            nNan = true;
        if(!pInf && !pNan && !nInf && !nNan)
        {
            //PCNormals->push_back(pcn->points[i]);
            pcn->points[i].x = 1000*pcn->points[i].x;
            pcn->points[i].y = 1000*pcn->points[i].y;
            pcn->points[i].z = 1000*pcn->points[i].z;
            //flipNormalViewpoint(vp,pcn->points[i]);
            PCNormals->push_back(pcn->points[i]);
        }
    }

    downSampling(PCNormals,Out,leaf_size);
}

//remove the background
void removeBG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold, const double colorThreshold)
{
    int width = scene->width;
    int height = scene->height;
    
    object->width = width;
    object->height = height;
    object->resize(width*height);
    object->is_dense = false;
    
    for(int i=0; i<width; i++)
    {
        for(int j=0; j<height; j++)
        {
            bool flag = false;
            
            if(isnan(scene->at(i,j).z))
                continue;
            if(corr(scene->at(i,j),empty_scene->at(i,j),positionThreshold,colorThreshold))
                flag = true;
          
            if(!flag)
            {
                object->at(i,j).x = scene->at(i,j).x;
                object->at(i,j).y = scene->at(i,j).y;
                object->at(i,j).z = scene->at(i,j).z;
                
            }
            else
            {
                object->at(i,j).x = std::numeric_limits<float>::quiet_NaN();
                object->at(i,j).y = std::numeric_limits<float>::quiet_NaN();
                object->at(i,j).z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
}

void removeBG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold)
{
    int width = scene->width;
    int height = scene->height;
    
    object->width = width;
    object->height = height;
    object->resize(width*height);
    object->is_dense = false;
    
    int n=0;

    for(int i=0; i<width; i++)
    {
        for(int j=0; j<height; j++)
        {
            bool flag = false;
            
            if(isnan(scene->at(i,j).z))
                continue;
            if(corr(scene->at(i,j),empty_scene->at(i,j),positionThreshold))
                flag = true;
          
            if(!flag)
            {
                object->at(i,j).x = scene->at(i,j).x;
                object->at(i,j).y = scene->at(i,j).y;
                object->at(i,j).z = scene->at(i,j).z;
            }
            else
            {
                n++;
                object->at(i,j).x = std::numeric_limits<float>::quiet_NaN();
                object->at(i,j).y = std::numeric_limits<float>::quiet_NaN();
                object->at(i,j).z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    //std::cout<<scene->points.size()<<std::endl;
    //std::cout<<"removed "<<n<<std::endl;
}


void removeBGWithPatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, const double positionThreshold, const double colorThreshold, const int patch_size)
{
    int width = scene->width;
    int height = scene->height;
    
    object->width = width;
    object->height = height;
    object->resize(width*height);
    object->is_dense = false;
    
    for(int i=0; i<width; i++)
    {
        for(int j=0; j<height; j++)
        {
            bool flag = false;
            
            if(isnan(scene->at(i,j).z))
            {
                object->at(i,j).x = object->at(i,j).y = object->at(i,j).z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            
            if(!isnan(empty_scene->at(i,j).z) && corr(scene->at(i,j),empty_scene->at(i,j),positionThreshold,colorThreshold))
                flag = true;
            if(!flag && searchInPatch(scene,empty_scene,i,j,patch_size,positionThreshold,colorThreshold))
                flag = true;
            if(!flag)
            {
                object->at(i,j).x = scene->at(i,j).x/1000.0;
                object->at(i,j).y = scene->at(i,j).y/1000.0;
                object->at(i,j).z = scene->at(i,j).z/1000.0;
            }
            else
                object->at(i,j).x = object->at(i,j).y = object->at(i,j).z = std::numeric_limits<float>::quiet_NaN();
        }
    }
}

bool searchInPatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_scene, int i, int j, const int patch_size, const double positionThreshold, const double colorThreshold)
{
    int left_bound=0, right_bound=0, up_bound=0, down_bound=0;
    int width=scene->width, height=scene->height;
    
    if((i-patch_size/2) < 0)
        left_bound = 0;
    else
        left_bound = i-patch_size/2;
    
    if((i+patch_size/2) > (width-1))
        right_bound = width-1;
    else
        right_bound = i+patch_size/2;
    
    if((j-patch_size/2) < 0)
        up_bound = 0;
    else
        up_bound = j-patch_size/2;
    
    if((j+patch_size/2) > (height-1))
        down_bound = height-1;
    else
        down_bound = j+patch_size/2;
    
   
    for(int w=left_bound; w<=right_bound; w++)
        for(int h=up_bound; h<=down_bound; h++)
        {
            if(!isnan(empty_scene->at(w,h).z) && corr(scene->at(i,j),empty_scene->at(w,h),positionThreshold,colorThreshold))
                return true;
        }
        
    return false;
}

//compute the color difference in the LAB space, see https://www.compuphase.com/cmetric.htm
bool corr(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, const double positionThreshold, const double colorThreshold)
{
    Eigen::Vector3f pos(p1.x-p2.x,p1.y-p2.y,p1.z-p2.z);
    double posDiff = pos.norm();
    int r1 = p1.r, g1 = p1.g, b1 = p1.b;
    int r2 = p2.r, g2 = p2.g, b2 = p2.b;
    double rmean = (r1+r2)/2.0;
    double dr = r1-r2;
    double dg = g1-g2;
    double db = b1-b2;
    double colorDiff = (double)sqrt((2.0+rmean/256.0)*dr*dr+4.0*dg*dg+(2.0+(255.0-rmean)/256.0)*db*db);
    if(posDiff<positionThreshold || colorDiff<colorThreshold)
        return true;
    
    return false;
}

bool corr(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2, const double positionThreshold)
{
    Eigen::Vector3f pos(p1.x-p2.x,p1.y-p2.y,p1.z-p2.z);
    double posDiff = pos.norm();

    if(posDiff<positionThreshold)
        return true;
    
    return false;
}

//transform point cloud with normals, and do not change the direction of normals
void transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Matrix4d& T)
{
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.topRightCorner<3,1>();
    
    for(int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointNormal p = cloud->points[i];
        
        Eigen::Vector3d pos(p.x,p.y,p.z);
        Eigen::Vector3d nor(p.x+p.normal_x,p.y+p.normal_y,p.z+p.normal_z);
        
        Eigen::Vector3d new_pos = R*pos+t;
        Eigen::Vector3d new_nor = R*nor+t;
        
        cloud->points[i].x = new_pos(0);
        cloud->points[i].y = new_pos(1);
        cloud->points[i].z = new_pos(2);
        
        Eigen::Vector3d n = new_nor-new_pos;
        n.normalize();
        cloud->points[i].normal_x = n(0);
        cloud->points[i].normal_y = n(1);
        cloud->points[i].normal_z = n(2);
    }
}

//compute the size of gear in x, y, z
Eigen::Vector3d getSize(const char *filename, const float scale_x, const float scale_y, const float scale_z)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    //
    vtkSmartPointer<vtkTransform> pTransform = vtkSmartPointer<vtkTransform>::New();
    pTransform->Scale(scale_x,scale_y,scale_z);
    vtkSmartPointer<vtkTransformPolyDataFilter> pTransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    pTransformPolyDataFilter->SetInputData(polydata);
    pTransformPolyDataFilter->SetTransform(pTransform);
    pTransformPolyDataFilter->Update();

    polydata = pTransformPolyDataFilter->GetOutput();

    double bound[6];
    polydata->GetBounds(bound);
    //std::cout<<"gear size: "<<bound[1]-bound[0]<<" "<<bound[3]-bound[2]<<" "<<bound[5]-bound[4]<<std::endl;

    Eigen::Vector3d bounds;

    bounds << bound[1]-bound[0], bound[3]-bound[2], bound[5]-bound[4];

    return bounds;
}

//remove plane
void removePlane(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr filtered)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointNormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.5);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a plane model for the given data.");
        return;
    }

    pcl::ExtractIndices<pcl::PointNormal> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered);

    pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
    outrem.setInputCloud(filtered);
    outrem.setRadiusSearch(10);
    outrem.setMinNeighborsInRadius(10);
    outrem.filter(*filtered);

    /*
    pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
    sor.setInputCloud(filtered);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);*/

}

//cluster point cloud, compute OBB, and find corresponding object
void computeBBOX(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::Vector3d& bounds, Eigen::Vector3d& box_size, pcl::PointNormal& obj_centre)
{
    std::vector<pcl::PointIndices> ece_inlier;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ece;

    double model_diameter = std::sqrt(bounds(0)*bounds(0)+bounds(1)*bounds(1)+bounds(2)*bounds(2));

    sortEigenVector(bounds);

    ece.setInputCloud(cloud);
    ece.setClusterTolerance(5.0);
    ece.setMinClusterSize(50);
    ece.setMaxClusterSize(2000);
    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);

    //for visualization
    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Object Detection"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointNormal> (cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud, 0, 255, 0), "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    */

    double min_score = 100;
    for (int i = 0; i < ece_inlier.size(); ++i)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointNormal>);
        std::vector<int> inlier_ext = ece_inlier[i].indices;
        copyPointCloud(*cloud, inlier_ext, *cluster_cloud);

        pcl::PointNormal min_point, max_point;

        pcl::getMinMax3D(*cluster_cloud, min_point, max_point);

        pcl::MomentOfInertiaEstimation<pcl::PointNormal> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();

        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        pcl::PointNormal min_point_obb;
        pcl::PointNormal max_point_obb;
        pcl::PointNormal position_obb;

        Eigen::Matrix3f rotational_matrix_obb;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        float major_value, middle_value, minor_value;

        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getOBB(min_point_obb, max_point_obb, position_obb, rotational_matrix_obb);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);

        Eigen::Vector3d obb_size(max_point_obb.x-min_point_obb.x, max_point_obb.y - min_point_obb.y, max_point_obb.z - min_point_obb.z);
        sortEigenVector(obb_size);

        //std::cout<<"size: "<<obb_size(0)<<" "<<obb_size(1)<<" "<<obb_size(2)<<std::endl;

        double delta_edge = std::fabs(bounds(0)-obb_size(0))+std::fabs(bounds(1)-obb_size(1))+std::fabs(bounds(2)-obb_size(2));
        double edge_score = delta_edge/(bounds(0)+bounds(1)+bounds(2));

        double cluster_diameter = std::sqrt(obb_size(0)*obb_size(0)+obb_size(1)*obb_size(1)+obb_size(2)*obb_size(2));
        double diameter_score = std::fabs(cluster_diameter-model_diameter)/model_diameter;

        //std::cout<<"edge score: "<<edge_score<<std::endl;
        //std::cout<<"diameter score: "<<diameter_score<<std::endl;

        double score = edge_score*diameter_score;

        if(score<min_score)
        {
            min_score = score;
            box_size = obb_size;
            obj_centre = position_obb;
        }

        //
        /*
        Eigen::Vector3f position(position_obb.x, position_obb.y, position_obb.z);
        Eigen::Quaternionf quat(rotational_matrix_obb);

        std::stringstream ss;
        ss << i;
        viewer->addCube(position, quat, max_point_obb.x - min_point_obb.x, max_point_obb.y - min_point_obb.y, max_point_obb.z - min_point_obb.z, "OBB" + ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "OBB" + ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "OBB" + ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "OBB" + ss.str());
        viewer->setRepresentationToWireframeForAllActors();

        pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
        pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1)+mass_center(1), major_vector(2)+mass_center(2));
        pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1)+mass_center(1), middle_vector(2)+mass_center(2));
        pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1)+mass_center(1), minor_vector(2)+mass_center(2));

        viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector" + ss.str());
        viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector" + ss.str());
        viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector" + ss.str());*/
    }

    /*
    std::cout<<"bounds: "<<bounds.transpose()<<std::endl;
    std::cout<<"box size: "<<box_size.transpose()<<std::endl;
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    */

}

void sortEigenVector(Eigen::Vector3d &vec)
{
    Eigen::Vector3d vec_sorted;

    double max_num = 0;
    int max_ind = 0;
    for(int i=0; i<3; i++)
    {
        if(vec(i)>max_num)
        {
            max_num = vec(i);
            max_ind = i;
        }
    }
    double middle_num = 0;
    int middle_ind = 0;
    for(int i=0; i<3; i++)
    {
        if(i == max_ind)
            continue;
        if(vec(i)>middle_num)
        {
            middle_num = vec(i);
            middle_ind = i;
        }
    }
    double min_num = 0;
    for(int i=0; i<3; i++)
    {
        if(i==max_ind || i==middle_ind)
            continue;
        min_num = vec(i);
    }

    vec(0) = max_num;
    vec(1) = middle_num;
    vec(2) = min_num;
}


/***********************************************************************************/
//functions for finding the most similar gear, not important for this prooject
//get all the file names in a folder
int getFileNames(const std::string& dir, std::vector<std::string>& filenames)
{
    boost::filesystem::path path(dir);
    if(!boost::filesystem::exists(path))
        return -1;

    boost::filesystem::directory_iterator end_iter;
    for(boost::filesystem::directory_iterator iter(path); iter!=end_iter; iter++)
    {
        if(boost::filesystem::is_regular_file(iter->status()))
            filenames.push_back(iter->path().string());

        if(boost::filesystem::is_directory(iter->status()))
            getFileNames(iter->path().string(),filenames);
    }

    return filenames.size();
}
//find most similar gear based on gear teeth and score
std::vector<std::string> processSTL(const char *filename, const std::string& dir)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    //
    vtkSmartPointer<vtkTransform> pTransform = vtkSmartPointer<vtkTransform>::New();
    pTransform->Scale(10,10,10);
    vtkSmartPointer<vtkTransformPolyDataFilter> pTransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    pTransformPolyDataFilter->SetInputData(polydata);
    pTransformPolyDataFilter->SetTransform(pTransform);
    pTransformPolyDataFilter->Update();

    polydata = pTransformPolyDataFilter->GetOutput();

    double bound[6];
    polydata->GetBounds(bound);

    double model_size = (bound[1]-bound[0])*(bound[3]-bound[2]);

    int teeth = getTeeth(filename);

    std::vector<std::string> gear_names;
    getFileNames(dir,gear_names);

    std::vector<std::pair<double,std::string>> similarity;

    for(auto& name:gear_names)
    {
        if(getTeeth(name.c_str())==teeth)
        {
            std::cout<<"found"<<std::endl;
            double gear_size = getSize(name.c_str());
            double diff = std::fabs(model_size-gear_size);
            similarity.push_back(std::make_pair(diff,name));
        }
    }
    std::cout<<similarity.size()<<std::endl;

    std::sort(similarity.begin(),similarity.end(),[](const std::pair<double,std::string>& a, const std::pair<double,std::string>& b){return a.first<b.first;});

    std::vector<std::string> best_names;
    for(int i=0; i<2; i++)
        best_names.push_back(similarity[i].second);

    return best_names;
}

//count the teeth of gear
int getTeeth(cv::Mat& gear_im)
{
    cv::Mat gear = gear_im.clone();
    //cv::imshow("gear",gear);
    //cv::waitKey(0);
    cv::Mat gray,bw;
    cv::cvtColor(gear,gray,CV_BGR2GRAY);
    cv::threshold(gray,bw,150,255,CV_THRESH_BINARY_INV);
    cv::bitwise_not(bw,bw);
    //cv::imshow("bw",bw);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    //cout<<"contour size "<<contours.size()<<endl;

    cv::Mat contoursIm(bw.size(),CV_8U,cv::Scalar(255));
    cv::drawContours(contoursIm,contours,-1,cv::Scalar(0),-1);

    std::vector<cv::Moments> mu(contours.size());
    for(int i=0; i<contours.size(); i++)
    {
        mu[i] = cv::moments(contours[i],true);
    }
    std::vector<cv::Point2f> mc(contours.size());
    for(int i=0; i<contours.size(); i++)
    {
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
        //cout<<"test"<<" "<<cv::pointPolygonTest(contours[i],mc[i],true);
    }
    double rmax,rmin,r,r2;
    for(int i=0;i<contours.size();i++)
    {
        for(int j=0;j<contours[i].size();j++)
        {
            r2=pow(contours[i][j].x-mc[i].x,2)+pow(contours[i][j].y-mc[i].y,2);
            r=sqrt(r2);
            if(j==0)
                rmax=rmin=r;
            if(rmax<=r)
                rmax=r;
            if(rmin>=r)
                rmin=r;
        }
    }

    cv::Mat temp_im = contoursIm.clone();
    cv::circle(temp_im,mc[0],(rmax+rmin)/2,cv::Scalar(255),-1);
    cv::bitwise_not(temp_im,temp_im);
    //cv::imshow("teeth",temp_im);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contour;
    cv::findContours(temp_im,contour,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    int teeth = contour.size();
    return teeth;
}

int getTeeth(const char* filename)
{
    cv::Mat gear = modelToImg(filename);
    //cv::imshow("gear",gear);
    //cv::waitKey(0);
    cv::Mat gray,bw;
    cv::cvtColor(gear,gray,CV_BGR2GRAY);
    cv::threshold(gray,bw,150,255,CV_THRESH_BINARY_INV);
    cv::bitwise_not(bw,bw);
    //cv::imshow("bw",bw);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bw,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    //cout<<"contour size "<<contours.size()<<endl;

    cv::Mat contoursIm(bw.size(),CV_8U,cv::Scalar(255));
    cv::drawContours(contoursIm,contours,-1,cv::Scalar(0),-1);

    std::vector<cv::Moments> mu(contours.size());
    for(int i=0; i<contours.size(); i++)
    {
        mu[i] = cv::moments(contours[i],true);
    }
    std::vector<cv::Point2f> mc(contours.size());
    for(int i=0; i<contours.size(); i++)
    {
        mc[i] = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
        //cout<<"test"<<" "<<cv::pointPolygonTest(contours[i],mc[i],true);
    }
    double rmax,rmin,r,r2;
    for(int i=0;i<contours.size();i++)
    {
        for(int j=0;j<contours[i].size();j++)
        {
            r2=pow(contours[i][j].x-mc[i].x,2)+pow(contours[i][j].y-mc[i].y,2);
            r=sqrt(r2);
            if(j==0)
                rmax=rmin=r;
            if(rmax<=r)
                rmax=r;
            if(rmin>=r)
                rmin=r;
        }
    }

    cv::Mat temp_im = contoursIm.clone();
    cv::circle(temp_im,mc[0],(rmax+rmin)/2,cv::Scalar(255),-1);
    cv::bitwise_not(temp_im,temp_im);
    //cv::imshow("teeth",temp_im);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contour;
    cv::findContours(temp_im,contour,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    int teeth = contour.size();
    return teeth;
}

//compute the size of the gear
double getSize(const char* filename)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    double bound[6];
    polydata->GetBounds(bound);
    std::cout<<"gear size: "<<bound[1]-bound[0]<<" "<<bound[3]-bound[2]<<" "<<bound[5]-bound[4]<<std::endl;

    //double gear_size = (bound[1]-bound[0])*(bound[3]-bound[2])*(bound[5]-bound[4]);
    double gear_size = (bound[1]-bound[0])*(bound[3]-bound[2]);

    return gear_size;
}
//compute the volumn of the gear
double getVolume(const char* filename, const float scale, const float leaf_size, double& area)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(filename,mesh);

    pcl::io::mesh2vtk(mesh,polydata);

    vtkSmartPointer<vtkTransform> pTransform = vtkSmartPointer<vtkTransform>::New();
    pTransform->Scale(scale,scale,scale);
    vtkSmartPointer<vtkTransformPolyDataFilter> pTransformPolyDataFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    pTransformPolyDataFilter->SetInputData(polydata);
    pTransformPolyDataFilter->SetTransform(pTransform);
    pTransformPolyDataFilter->Update();

    polydata = pTransformPolyDataFilter->GetOutput();

    vtkSmartPointer<vtkTriangleFilter> triF = vtkSmartPointer<vtkTriangleFilter>::New();
    triF->SetInputData(polydata);

    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triF->GetOutputPort());
    triangleMapper->Update();

    vtkSmartPointer<vtkMassProperties> polygonProperties = vtkSmartPointer<vtkMassProperties>::New();
    polygonProperties->SetInputData(triangleMapper->GetInput());
    polygonProperties->Update();
    area = polygonProperties->GetSurfaceArea();
    double vol = polygonProperties->GetVolume();

    //std::cout<<"volume "<<vol<<std::endl;

    double surface_vol = area*leaf_size;

    //std::cout<<"surface vol "<<surface_vol<<std::endl;
    return surface_vol;
}
//sampling on the model, number of points is approximately the same with the parameter "samples"
void samplingOnModel(const char* filename, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, const int samples, const float leaf_size)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    loadSTL(cloud,filename,100000,1,1,1);

    double area;
    double vol = getVolume(filename,1,leaf_size,area);
    double cube_vol = vol/samples;
    double cube_area = area/samples;

    float leaf_size1 = std::pow(cube_vol,1.0/3.0);
    float leaf_size2 = std::pow(cube_area,1.0/2.0);
    float leaf_size3 = (leaf_size1+leaf_size2)/2.0;
    std::cout<<"leaf size "<<leaf_size3<<std::endl;

    downSampling(cloud,PCNormals,leaf_size3);
}
//sampling on the gear model, the number of points is approximately the same with that of the siemens benchmark
void samplingOnModel(const char *filename, pcl::PointCloud<pcl::PointNormal>::Ptr PCNormals, Eigen::Vector3d &bounds, const float leaf_size)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    loadSTL(cloud,filename,100000,1,1,1);

    Eigen::Vector3d gear_size;
    gear_size = getSize(filename,1,1,1);
    double ratio = gear_size(0)/bounds(0);
    double leaf_x = leaf_size*ratio;
    double leaf_y = leaf_size*ratio;
    double s = (bounds(0)/bounds(2))/(gear_size(0)/gear_size(2));
    double leaf_z = leaf_size*ratio*s;

    pcl::VoxelGrid<pcl::PointNormal> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(leaf_x,leaf_y,leaf_z);
    vox.filter(*PCNormals);
}
//convert STL model to 2d image
cv::Mat modelToImg(const char *filename)
{
    vtkSTLReader* model = vtkSTLReader::New();
    model->SetFileName(filename);

    vtkPolyDataMapper* modelMapper = vtkPolyDataMapper::New();
    modelMapper->SetInputConnection(model->GetOutputPort());

    vtkLODActor* modelActor = vtkLODActor::New();
    modelActor->SetMapper(modelMapper);

    modelActor->GetProperty()->SetColor(1.0,1.0,1.0);

    //vtkRenderer* render = vtkRenderer::New();
    vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
    render->AddActor(modelActor);
    render->SetBackground(0.1,0.1,0.1);
    render->ResetCamera();
    render->GetActiveCamera()->Zoom(1.0);

    //vtkRenderWindow* renderWin = vtkRenderWindow::New();
    vtkSmartPointer<vtkRenderWindow> renderWin = vtkSmartPointer<vtkRenderWindow>::New();
    renderWin->AddRenderer(render);
    renderWin->SetSize(640,480);
    renderWin->OffScreenRenderingOn();
    renderWin->Render();
    vtkSmartPointer<vtkWindowToImageFilter> window2img = vtkSmartPointer<vtkWindowToImageFilter>::New();
    window2img->SetInput(renderWin);
    window2img->SetInputBufferTypeToRGB();
    window2img->ReadFrontBufferOff();
    window2img->Update();

    int dim[3];
    window2img->GetOutput()->GetDimensions(dim);
    cv::Mat renderedImg(dim[1],dim[0],CV_8UC3,window2img->GetOutput()->GetScalarPointer());

    cv::flip(renderedImg,renderedImg,0);

    return renderedImg.clone();
}


}
