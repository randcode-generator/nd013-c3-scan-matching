// #ifndef ICP
// #define ICP

// #include <pcl/io/pcd_io.h>
// #include <vector>
// #include <Eigen/Geometry>
// #include <Eigen/Core>
// #include <Eigen/SVD>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "helper.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct Pair{

	Point p1;
	Point p2;

	Pair(Point setP1, Point setP2)
		: p1(setP1), p2(setP2){}
};

Eigen::Matrix4d ICP(vector<int> associations, PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations, pcl::visualization::PCLVisualizer::Ptr& viewer);

//#endif