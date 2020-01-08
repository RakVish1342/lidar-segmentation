
std::vector<std::vector<float>> pcl2Vec(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::vector<std::vector<float>> vect;
    for(uint i=0; i<cloud->points.size(); ++i)
    {
        vect.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }
    return vect;
}


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> pt, int setId)
	:	point(pt), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    //void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		//Tree is empty
		if(*node==NULL)
		{
			//std::cout << "IN NULL" << std::endl;
			//std::cout << id << ", " << point[0] << ", " << point[1] << std::endl;
			*node = new Node(point, id);
		}
		else
		{
			//Calculate current dimension
			uint cd = depth % 3;

			if(point[cd] < ((*node)->point[cd]))
			{
				//std::cout << "IN LEFT" << std::endl;
				//std::cout << id << ", " << point[0] << ", " << point[1] << std::endl;
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				//std::cout << "IN RIGHT" << std::endl;
				//std::cout << id << ", " << point[0] << ", " << point[1] << std::endl;
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}	
	}

    //void insert(pcl::PointXYZI point, int id)
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if(node!=NULL)
		{
			if( (node->point[0]>=(target[0]-distanceTol)) && (node->point[0]<=(target[0]+distanceTol)) && 
				(node->point[1]>=(target[1]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol)) &&
				(node->point[2]>=(target[2]-distanceTol)) && (node->point[2]<=(target[2]+distanceTol)) )
			{
				float distance = sqrt( (node->point[0]-target[0])*(node->point[0]-target[0]) + 
					(node->point[1]-target[1])*(node->point[1]-target[1]) + 
					(node->point[2]-target[2])*(node->point[2]-target[2]) );
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			int cd = depth % 3; //current dimension
			//check across boundary
			if((target[cd]-distanceTol) < node->point[cd])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target[cd]+distanceTol) > node->point[cd])
				searchHelper(target, node->right, depth+1, distanceTol, ids);			
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
		clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while(i < points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;
}
