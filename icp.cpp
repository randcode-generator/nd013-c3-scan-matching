#include "icp.h"

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){
    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    
    // align source with starting pose
    Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
    PointCloudT::Ptr transformSource (new PointCloudT); 
    pcl::transformPointCloud (*source, *transformSource, initTransform);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (transformSource);
    icp.setInputTarget (target);
    icp.setMaxCorrespondenceDistance (1);
  	icp.setTransformationEpsilon (1e-10);
	icp.setEuclideanFitnessEpsilon (0.01);

    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
    icp.align (*cloud_icp);
    
    if (icp.hasConverged ())
    {
      	std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        transformation_matrix =  transformation_matrix * initTransform;
        
        return transformation_matrix;
    }
    else
        cout << "WARNING: ICP did not converge" << endl;
    return transformation_matrix;
}

Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations){
	
	pcl::console::TicToc time;
	time.tic ();

	Eigen::Matrix4f init_guess = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();

  	// Setting max number of registration iterations.
  	ndt.setMaximumIterations (iterations);
	ndt.setInputSource (source);
  	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt (new pcl::PointCloud<pcl::PointXYZ>);
  	ndt.align (*cloud_ndt, init_guess);

	cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () << " score: " << ndt.getFitnessScore () <<  " time: " << time.toc() <<  " ms" << endl;

	Eigen::Matrix4d transformation_matrix = ndt.getFinalTransformation ().cast<double>();

	return transformation_matrix;
}