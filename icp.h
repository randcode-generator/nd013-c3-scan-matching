#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "helper.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);