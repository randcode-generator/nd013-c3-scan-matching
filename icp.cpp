#include "icp.h"

vector<Pair> estimations;

vector<Pair> PairPoints(vector<int> associations, PointCloudT::Ptr target, PointCloudT::Ptr source, bool render, pcl::visualization::PCLVisualizer::Ptr& viewer){

	vector<Pair> pairs;

	// TODO: loop through each source point and using the corresponding associations append a Pair of (source point, associated target point)
  	int index = 0;
  	for(PointT point : source->points ){
      int i = associations[index];
      if( i >= 0) {
        PointT association = (*target)[i];
        if(render){
          viewer->removeShape(to_string(index));
          renderRay(viewer, Point(point.x, point.y,0), Point(association.x, association.y,0), to_string(index), Color(0,1,0));
        }
        pairs.push_back(Pair(Point(point.x, point.y,0), Point(association.x, association.y,0)) );
      }
      index++;
    }

	return pairs;
}
Eigen::Matrix4d ICP(vector<int> associations, PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations, pcl::visualization::PCLVisualizer::Ptr& viewer){
    // align source with starting pose
    Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
    PointCloudT::Ptr transformSource (new PointCloudT);
    pcl::transformPointCloud (*source, *transformSource, initTransform);
    
    vector<Pair> pairs = PairPoints(associations, target, transformSource, true, viewer);

    //cout << "score is " << Score(pairs, Eigen::MatrixXd::Identity(4,4) ) << endl;

    Eigen::MatrixXd X(2,pairs.size());
    Eigen::MatrixXd Y(2,pairs.size());
    Eigen::MatrixXd P(2,1);
    P << Eigen::MatrixXd::Zero(2,1);
    Eigen::MatrixXd Q(2,1);
    Q << Eigen::MatrixXd::Zero(2,1);

    for(Pair pair : pairs){
        P(0,0) += pair.p1.x;
        P(1,0) += pair.p1.y;

        Q(0,0) += pair.p2.x;
        Q(1,0) += pair.p2.y;
    }
    P(0,0) = P(0,0)/pairs.size();
    P(1,0) = P(1,0)/pairs.size();

    Q(0,0) = Q(0,0)/pairs.size();
    Q(1,0) = Q(1,0)/pairs.size();
    int index = 0;
    for(Pair pair : pairs){
        X(0,index) = pair.p1.x - P(0,0);
        X(1,index) = pair.p1.y - P(1,0);

        Y(0,index) = pair.p2.x - Q(0,0);
        Y(1,index) = pair.p2.y - Q(1,0);
        index++;
    }

    // compute best R and t from using SVD
    Eigen::MatrixXd S  = X * Y.transpose();
    JacobiSVD<MatrixXd> svd(S, ComputeFullV | ComputeFullU);
    Eigen::MatrixXd D;
    D.setIdentity(svd.matrixV().cols(), svd.matrixV().cols());
    D(svd.matrixV().cols()-1,svd.matrixV().cols()-1) = (svd.matrixV() * svd.matrixU().transpose() ).determinant();

    Eigen::MatrixXd R  = svd.matrixV() * D * svd.matrixU().transpose();
    Eigen::MatrixXd t  = Q - R * P;

    Eigen::Matrix4d transformation_matrix;
    transformation_matrix << Eigen::MatrixXd::Identity(4,4);

    transformation_matrix(0,0) = R(0,0);
    transformation_matrix(0,1) = R(0,1);
    transformation_matrix(1,0) = R(1,0);
    transformation_matrix(1,1) = R(1,1);
    transformation_matrix(0,3) = t(0,0);
    transformation_matrix(1,3) = t(1,0);

    //cout << "score is " << Score(pairs, transformation_matrix ) << endl;

    //cout << transformation_matrix << endl;

    estimations = pairs;
    transformation_matrix =  transformation_matrix * initTransform;
    return transformation_matrix;
}