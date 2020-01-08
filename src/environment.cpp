/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <unordered_set>


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    //bool renderScene = true;
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld = lidar->scan();
    // cout << lidar << endl;
    // cout << &lidar << endl;
    // //cout << (*lidar).scan() << endl;
    // cout << (*lidar).scan() << endl;

    // renderRays(viewer, 
    //             lidar->position, 
    //             ptcld);
    
    renderPointCloud(viewer, ptcld, "Test Point Cloud");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(ptcld, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId=0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new  ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(
        "../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    std::cout << "inputCloud size: " << inputCloud->points.size() << std::endl;
    
    // Eigen::Vector4f minPoint(-10.0, -6.0, -10.0, -200);
    // Eigen::Vector4f maxPoint(-5.0, 7.8, 10.0, 200);
    Eigen::Vector4f minPoint(-10.0, -6.0, -10.0, -200);
    Eigen::Vector4f maxPoint(10.0, 7.8, 10.0, 200);    
    float voxel_cube_size = 0.35;

    //pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = ProcessPointClouds<pcl::PointXYZI>::FilterCloud(inputCloud, 2.0, minPoint, maxPoint );
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, voxel_cube_size, minPoint, maxPoint );
    //renderPointCloud(viewer, filteredCloud, "filteredCloud");
    std::cout << "filteredCloud size: " << filteredCloud->points.size() << std::endl;
    return filteredCloud;
}



int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cityBlockCloud = cityBlock(viewer); //Voxeled/compressed cloud

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new  ProcessPointClouds<pcl::PointXYZI>();

/*
    //int maxIterPlaneSeg = cityBlockCloud->points.size()*0.4;
    int maxIterPlaneSeg = 5000;
    float distThreshPlaneSeg = 0.5;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudPair = pointProcessorI->MySegmentPlane(cityBlockCloud, maxIterPlaneSeg, distThreshPlaneSeg, viewer);
    //pointProcessorI->MySegmentPlane(cityBlockCloud, maxIterPlaneSeg, distThreshPlaneSeg);
    renderPointCloud(viewer, cloudPair.first, "Plane Cloud", Color(1,1,1));
    renderPointCloud(viewer, cloudPair.second, "Obst Cloud", Color(0,1,0));
    //$$ WAIIIT Should I interchange obst and plane??
*/

    int maxIterPlaneSeg = 5000;
    float distThreshPlaneSeg = 1.1;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudVec = pointProcessorI->MySegmentPlane(cityBlockCloud, maxIterPlaneSeg, distThreshPlaneSeg, viewer);
    std::cout << "cityBlockCloud Num:" << cityBlockCloud->points.size();
    renderPointCloud(viewer, cloudVec[0], "Plane Cloud", Color(0,1,0));
    renderPointCloud(viewer, cloudVec[1], "Obst Cloud", Color(1,0,0));
    renderPointCloud(viewer, cloudVec[2], "Plane Pts Cloud", Color(1,1,1));

    //$$ WAIIIT Should I interchange obst and plane??

    // //Separate Clouds
    // std::unordered_set<int> converseGroundInliers = pointProcessorI->MySegmentPlaneInliers(cityBlockCloud, maxIterPlaneSeg, distThreshPlaneSeg);
    // std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->SeparateClouds(converseGroundInliers, cityBlockCloud);
    // renderPointCloud(viewer, segResult.first, "idk");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}