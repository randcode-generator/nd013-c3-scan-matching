#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/console/time.h>   // TicToc

#include "helper.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);
Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations);