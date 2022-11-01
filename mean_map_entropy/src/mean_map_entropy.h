//============================================================================
// Name        : MeanMapEntropy.cpp
// Author      : David Droeschel & Jan Razlaw
// Version     :
// Copyright   :
// Description : Calculates Mean Map Entropy and Mean Plane Variance of a point cloud
//============================================================================

#include <omp.h>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>


struct PointTypeWithEntropy
{
	PCL_ADD_POINT4D;                  	// preferred way of adding a XYZ + padding
	float entropy;
	float planeVariance;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   	// make sure our new allocators are aligned
} EIGEN_ALIGN16;                    		// enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointTypeWithEntropy,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(float, entropy, entropy)
		(float, planeVariance, planeVariance)
)

typedef pcl::PointXYZ PointT;


double computeEntropy( pcl::PointCloud< PointT >::Ptr cloud );


double computePlaneVariance( pcl::PointCloud< PointT >::Ptr cloud );

void computeEntropyMain(pcl::PointCloud< PointT >::Ptr inputCloud, 
        pcl::PointCloud< PointTypeWithEntropy >::Ptr outputCloud, 
		int stepSize = 1, double radius = 0.3, 
		bool punishSolitaryPoints = false, int minNeighbors = 15);

void save(pcl::PointCloud< PointTypeWithEntropy >::Ptr outputCloud, const std::string& pcdFilename);
