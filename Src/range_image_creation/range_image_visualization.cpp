// #include <iostream>

// #include <boost/thread/thread.hpp>

// #include <pcl/common/common_headers.h>
// #include <pcl/range_image/range_image.h>
// #include <pcl/range_image/range_image_planar.h>
// #include <pcl/visualization/range_image_visualizer.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>

// #include <vtkImageReader2Factory.h>
// #include <vtkImageReader2.h>
// #include <vtkImageData.h>
// #include <vtkImageFlip.h>
// #include <vtkPolyLine.h>

#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>

typedef pcl::PointXYZ PointType;
using namespace pcl;
using namespace std;

void
mySaveRangeImagePlanarFilePNG (const string &file_name, const RangeImagePlanar& range_image)
{
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
  image->SetDimensions(range_image.width, range_image.height, 1);

  image->SetNumberOfScalarComponents(1);
  image->SetScalarTypeToFloat();
  image->AllocateScalars();

  int* dims = image->GetDimensions();

  for (int y = 0; y < dims[1]; y++)
    {
    for (int x = 0; x < dims[0]; x++)
      {
      float* pixel = static_cast<float*>(image->GetScalarPointer(x,y,0));
      pixel[0] = range_image(x,y).range;
      if (pixel[0] < 0) pixel[0] = 0.0f; // Get rid of ranges in -Inf or +Inf. 
      }
    }

  // Compute the scaling
  // cout << "img scalar range " << image->GetScalarRange()[0] << " " << image->GetScalarRange()[1] << endl;

  float oldRange = static_cast<float> (image->GetScalarRange()[1] - image->GetScalarRange()[0]);
  float newRange = 255; // We want the output [0,255]

  vtkSmartPointer<vtkImageShiftScale> shiftScaleFilter = vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleFilter->SetOutputScalarTypeToUnsignedChar();

  shiftScaleFilter->SetInputConnection(image->GetProducerPort());

  shiftScaleFilter->SetShift(-1.0f * image->GetScalarRange()[0]); // brings the lower bound to 0
  shiftScaleFilter->SetScale(newRange/oldRange);
  shiftScaleFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(file_name.c_str());
  writer->SetInputConnection(shiftScaleFilter->GetOutputPort());
  writer->Write();
}


// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{

  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, *point_cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      // printUsage (argv[0]);
      return 0;
    }
  }
  else
  {
    std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud->points.push_back (point);
      }
    }
    point_cloud->width = (int) point_cloud->points.size ();  point_cloud->height = 1;
  }

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::cout << "Loading visualization...\n";
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud(point_cloud, "point cloud");

  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  int width = 640;
  int height = 480;

  float fx_d = 5.8262448167737955e+02;
  float fy_d = 5.8269103270988637e+02;
  float cx_d = 3.1304475870804731e+02;
  float cy_d = 2.3844389626620386e+02;
  Eigen::Affine3f sensor_pose = viewer->getViewerPose();

  RangeImagePlanar::Ptr range_image(new pcl::RangeImagePlanar());
  range_image->createFromPointCloudWithFixedSize(*point_cloud, width, height,
                                  cx_d, cy_d, fx_d, fy_d, sensor_pose);

  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (*range_image);
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer->spinOnce ();
    pcl_sleep (1);

    Eigen::Affine3f sensor_pose = viewer->getViewerPose();
    range_image->createFromPointCloudWithFixedSize(*point_cloud, width, height,
                                  cx_d, cy_d, fx_d, fy_d, sensor_pose);
    range_image_widget.showRangeImage (*range_image);
    mySaveRangeImagePlanarFilePNG ("range.png", *range_image);
    
    // if (live_update)
    // {
    //   scene_sensor_pose = viewer.getViewerPose();
    //   range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
    //                                     pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
    //                                     scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
    //   range_image_widget.showRangeImage (range_image);
    // }
  }
}