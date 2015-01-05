#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/ply_io.h>


using namespace pcl;
// Types
typedef PointXYZRGB PointT;
typedef PointCloud<PointT> PointCloudT;
// typedef PointNormal PointNT;
// typedef PointCloud<PointNT> PointNCloudT;
// typedef PointXYZL PointLT;
// typedef PointCloud<PointLT> PointLCloudT;
// typedef Normal NormalT;
// typedef PointCloud<NormalT> NormalCloudT;


int
main (int argc, char** argv)
{

  if(argc != 2)
  {
  std::cout << "Usage: " << argv[0] << "  Filename(.pcd)" << std::endl;
  return EXIT_FAILURE;
  }
 
  std::string inputFilename = argv[1];

  // Load input file into a PointCloud<T> with an appropriate type
  PointCloudT::Ptr cloud (new PointCloudT);
  PCLPointCloud2 cloud_blob;
  io::loadPCDFile (inputFilename, cloud_blob);
  fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  NormalEstimation<PointT, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZRGBNormal>);
  concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  search::KdTree<PointXYZRGBNormal>::Ptr tree2 (new search::KdTree<PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  GreedyProjectionTriangulation<PointXYZRGBNormal> gp3;
  PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.1);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  io::savePLYFile ("mesh.ply", triangles);

  // Finish
  return (0);
}