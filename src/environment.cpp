#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <unordered_set>
#include "kdtree.cpp"

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
    // Just playing with pointers
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, 
                ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    std::cout << "inputCloud size: " << inputCloud->points.size() << std::endl;

    Eigen::Vector4f minPoint(-15.0, -6.0, -10.0, -200);
    Eigen::Vector4f maxPoint(15.0, 7.8, 10.0, 200);    
    float voxel_cube_size = 0.35;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 
        voxel_cube_size, minPoint, maxPoint);
    //renderPointCloud(viewer, filteredCloud, "filteredCloud");

    std::cout << "filteredCloud size: " << filteredCloud->points.size() << std::endl;
    //return filteredCloud;

    int maxIterPlaneSeg = 500;
    float distThreshPlaneSeg = 1.3;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudVec = pointProcessorI->MySegmentPlane(filteredCloud, maxIterPlaneSeg, distThreshPlaneSeg, viewer);
    std::cout << "filteredCloud Num:" << filteredCloud->points.size();
    //renderPointCloud(viewer, cloudVec[0], "Plane Cloud", Color(0,1,0));
    renderPointCloud(viewer, cloudVec[1], "Obst Cloud", Color(1,0,0));
    //renderPointCloud(viewer, cloudVec[2], "Plane Pts Cloud", Color(1,1,1));
    //$$ WAIIIT Should I interchange obst and plane??

    KdTree* tree = new KdTree;

    // $$DO later: Inserting root node from the median value (of x dimension) 
    // for faster search response from tree
    std::vector<std::vector<float>> obstPoints = pcl2Vec(cloudVec[0]);
    for (int i=0; i<obstPoints.size(); ++i)
    {
        tree->insert(obstPoints[i],i);
    }

    float distThreshClust = 0.6;
    int minClusterPts = 6;
    std::vector<std::vector<int>> clusters = euclideanCluster(obstPoints, tree, distThreshClust);

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,1,0), Color(0,1,1), Color(1,0,1)};

    // Now just storing clusters as XYZ and not XYZI anymore
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new  ProcessPointClouds<pcl::PointXYZ>();
    for(std::vector<int> cluster : clusters)
  	{
        // assign to cluster only if number of pts is significant
        if(cluster.size() > minClusterPts)
        {
            for(int indice: cluster)
            {
                clusterCloud->points.push_back( pcl::PointXYZ(obstPoints[indice][0], 
                    obstPoints[indice][1], obstPoints[indice][2]) );
            }
            renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId), colors[clusterId%colors.size()]);
            Box box = pointProcessor->BoundingBox(clusterCloud);
            renderBox(viewer, box, clusterId);
            clustersCloud.push_back(clusterCloud);
        }
        clusterCloud->clear();
        ++clusterId;
  	}
  	if(clusters.size()==0)
    {
        std::cout << "No Clusters Found." << std::endl;
    }
    else
    {
        std::cout << "Clusters Found:" << clustersCloud.size() << std::endl;
    }

}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new  ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce();
    }
}