// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <math.h>
#include <cmath>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    //return cloud;
    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

// template<typename PointT>
// std::unordered_set<int> ProcessPointClouds<PointT>::MySegmentPlaneInliers(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
// 	std::unordered_set<int> inliersResult;
//     std::unordered_set<int> converseInliersResult;
//     pcl::PointIndices::Ptr conv {new pcl::PointIndices};

// 	srand(time(NULL));

// 	std::unordered_set<int> inliers;	
// 	// For max iterations
// 	while(maxIterations--)
// 	{
// 		// Randomly sample subset to get 3 points
// 		while(inliers.size() < 3) //ensure atleast tow unique samples have been added to this set.
// 			inliers.insert(rand()%(cloud->points.size()));
		
// 		float x1, y1, z1;
// 		float x2, y2, z2;
// 		float x3, y3, z3;

// 		auto itr = inliers.begin();
// 		x1 = cloud->points[*itr].x;
// 		y1 = cloud->points[*itr].y;
// 		z1 = cloud->points[*itr].z;
// 		itr++;
// 		x2 = cloud->points[*itr].x;
// 		y2 = cloud->points[*itr].y;
// 		z2 = cloud->points[*itr].z;
// 		itr++;
// 		x3 = cloud->points[*itr].x;
// 		y3 = cloud->points[*itr].y;
// 		z3 = cloud->points[*itr].z;
		
// 		float cross_i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
// 		float cross_j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
// 		float cross_k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

// 		float a = cross_i;
// 		float b = cross_j;
// 		float c = cross_k;
// 		float d = -(cross_i*x1 + cross_j*y1 + cross_k*z1);

// 	    // Measure distance between every point and fitted line
//     	// If distance is smaller than threshold count it as inlier
// 		for(int index = 0; index < cloud->points.size(); index++)
// 		{
// 			if(inliers.count(index)>0)
//             {
// 				continue;
//             }

// 			PointT point = cloud->points[index]; 
// 			float x4 = point.x;
// 			float y4 = point.y;
// 			float z4 = point.z;

// 			float d = fabs((a*x4 + b*y4 + c*z4)/sqrt(a*a + b*b + c*c));

// 			if(d <= distanceThreshold)
// 				inliers.insert(index);
// 		}

// 		if(inliers.size() > inliersResult.size())
// 		{
// 			inliersResult = inliers;
//             //conv ---feed this with a vector of ints
// 		}
// 	}
// 	// Create/Separate Plane and Obstacle Point Cloud
//     //First = planeCloud, Second = obstCloud

//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> cloudPair;
//     typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
//     typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
//     for(int idx = 0; idx < cloud->points.size(); ++idx)
//     {
//         if(!inliersResult.count(idx))
//         {
//             converseInliersResult.insert(idx);
//         }
//     }
//     return converseInliersResult;
// }



// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, pcl::visualization::PCLVisualizer::Ptr &viewer)
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
	std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliersPlaneResult;
	srand(time(NULL));

	std::unordered_set<int> inliers;
    std::unordered_set<int> inliersPlane; //original 3 points used to define plan
    float planeAngleResult = 0;
	// For max iterations
	while(maxIterations--)
	{
        inliers.clear();
        inliersPlane.clear();
		// Randomly sample subset to get 3 points
		while(inliers.size() < 3) //ensure atleast tow unique samples have been added to this set.
        {
            int num = rand()%(cloud->points.size());
			inliers.insert(num);
            inliersPlane.insert(num);
        }
		
		float x1, y1, z1;
		float x2, y2, z2;
		float x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		
		float cross_i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float cross_j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float cross_k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

		float a = cross_i;
		float b = cross_j;
		float c = cross_k;
		float d = -(cross_i*x1 + cross_j*y1 + cross_k*z1);

        
        float planeAngle = std::acos((a*0 + b*0 + 1*c)/(std::sqrt(a*a+b*b+c*c)*std::sqrt(1)));
        float angleThresh = 1*(3.1415/180);
        // Acept 3 points for plane only if in threshold. Else neglect points and try new points
        if (planeAngle > angleThresh)
        {
            // Can probably comment clearning statements, since clearing happens in each loop now

            //std::cout << "NO" << std::endl;
            inliers.clear();
            inliersPlane.clear();
            viewer->removeShape("plane");
            continue;
        }
        else
        {
            //std::cout << "YES" << std::endl;
            pcl::ModelCoefficients coeffsPlane;
            coeffsPlane.values.push_back(a);
            coeffsPlane.values.push_back(b);
            coeffsPlane.values.push_back(c);
            coeffsPlane.values.push_back(d);
            viewer->addPlane(coeffsPlane, "plane");            

            // Measure distance between every point and fitted line
            // If distance is smaller than threshold count it as inlier
            for(int index = 0; index < cloud->points.size(); index++)
            {
                if(inliers.count(index)>0)
                {
                    continue;
                }

                PointT point = cloud->points[index]; 
                float x4 = point.x;
                float y4 = point.y;
                float z4 = point.z;

                float d = fabs((a*x4 + b*y4 + c*z4)/sqrt(a*a + b*b + c*c));

                if(d <= distanceThreshold)
                    inliers.insert(index);
            }

            if(inliers.size() > inliersResult.size())
            {
                inliersResult = inliers;
                inliersPlaneResult = inliersPlane;
                planeAngleResult = planeAngle;
            }
        }
	}
	// Create/Separate Plane and Obstacle Point Cloud
    //First = planeCloud, Second = obstCloud

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> cloudPair;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudVec(3);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planePtsCloud(new pcl::PointCloud<PointT>());
    for(int idx = 0; idx < cloud->points.size(); ++idx)
    {
        if(inliersPlaneResult.count(idx))
        {
            
            planePtsCloud->points.push_back(cloud->points[idx]);
        }
        if(inliersResult.count(idx))
        {
            //cloudPair.first->points.push_back(cloud->points[idx]);
            planeCloud->points.push_back(cloud->points[idx]);
        }
        else
        {
            //cloudPair.second->points.push_back(cloud->points[idx]);
            obstCloud->points.push_back(cloud->points[idx]);
        }
            
    }
    std::cout << "planeCloud Num:" << planeCloud->points.size() << std::endl;
    std::cout << "obstCloud Num:" << obstCloud->points.size() << std::endl;
    std::cout << "planePtsCloud Num:" << planePtsCloud->points.size() << std::endl;
    std::cout << "planeAngle Num:" << planeAngleResult*(180/3.1415) << std::endl;

    //renderPointCloud(viewer, cityplaneCloud, "Plane Cloud");

    // cloudPair.first = planeCloud;
    // cloudPair.second = obstCloud;
    // return cloudPair;
    cloudVec[0] = planeCloud;
    cloudVec[1] = obstCloud;
    cloudVec[2] = planePtsCloud;
    return cloudVec;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
 	//pcl::PointIndices::Ptr inliers;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr, 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);

    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}