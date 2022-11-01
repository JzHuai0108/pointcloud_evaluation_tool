
#include <omp.h>

#include <iostream>

#include "mean_map_entropy.h"

int main( int argc, char** argv ) {
	pcl::PointCloud< PointT >::Ptr inputCloud (new pcl::PointCloud< PointT >);
	pcl::PointCloud< PointTypeWithEntropy >::Ptr outputCloud (new pcl::PointCloud< PointTypeWithEntropy >);

	// get pointcloud
	std::vector<int> fileIndices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (pcl::io::loadPCDFile< PointT > (argv[fileIndices[0]], *inputCloud) == -1)
	{
		PCL_ERROR ("Couldn't read file.\n");
		return (-1);
	}

	// get parameters if given
	int stepSize = 1;
	double radius = 0.3;
	int minNeighbors = 15;

	pcl::console::parse_argument (argc, argv, "-stepsize", stepSize);
    pcl::console::parse_argument (argc, argv, "-radius", radius);
	bool punishSolitaryPoints = pcl::console::find_switch (argc, argv, "-punishSolitaryPoints");
    pcl::console::parse_argument (argc, argv, "-minNeighbors", minNeighbors);
    std::cout << "Point cloud size " << inputCloud->size() << std::endl;
    computeEntropyMain(inputCloud, outputCloud, stepSize, radius, punishSolitaryPoints, minNeighbors);
    
	std::string pcdFilename = argv[fileIndices[0]];
    save(outputCloud, pcdFilename);
	return 0;
}
