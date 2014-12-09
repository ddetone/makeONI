// #include <iostream>

// #include <boost/thread/thread.hpp>

// #include <pcl/common/common_headers.h>
// #include <pcl/range_image/range_image.h>
// #include <pcl/range_image/range_image_planar.h>
// #include <pcl/visualization/range_image_visualizer.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>

// #include <pcl/common/time.h>
// #include <pcl/common/io.h>
// #include <pcl/console/parse.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/png_io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/segmentation/supervoxel_clustering.h>
// #include <pcl/range_image/range_image_planar.h>
// #include <pcl/visualization/range_image_visualizer.h>

// #include <vtkImageReader2Factory.h>
// #include <vtkImageReader2.h>
// #include <vtkImageData.h>
// #include <vtkImageFlip.h>
// #include <vtkPolyLine.h>

#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkPNGWriter.h>

#include <vtkPolyData.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDepthSortPolyData.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>

#include <vtkObjectFactory.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <string.h>

# define VTK_CREATE(type, nom) vtkSmartPointer<type> nom = vtkSmartPointer<type>::New()

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
 
    virtual void OnKeyPress() 
    {
      // Get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();
 
      // Output the key that was pressed
      std::cout << "Pressed " << key << std::endl;
 
      // Handle an arrow key
      if(key == "Up")
        {
          // read depth image from vtk Window
        VTK_CREATE (vtkWindowToImageFilter, filterDepth);
        filterDepth->SetInput (rwi->GetRenderWindow());
        filterDepth->SetMagnification( 1 );
        filterDepth->SetInputBufferTypeToZBuffer();
        filterDepth->ReadFrontBufferOff();
        filterDepth->Update ();

        VTK_CREATE(vtkImageData, imdepth);
        imdepth = filterDepth->GetOutput();
        int dimsdepth[3];
        imdepth->GetDimensions(dimsdepth);

        VTK_CREATE(vtkImageShiftScale, scalerDepth);
        // scaler->SetOutputScalarTypeToUnsignedShort();
        // scaler->SetInputConnection(imdepth->GetProducerPort());
        // scaler->SetShift(-1.0f * image->GetScalarRange()[0]); // brings the lower bound to 0
        // scaler->SetScale(newRange/oldRange);
        scalerDepth->SetOutputScalarTypeToUnsignedChar(); 
        scalerDepth->SetInputConnection(filterDepth->GetOutputPort()); 
        scalerDepth->SetShift(0); 
        scalerDepth->SetScale(-255);
        scalerDepth->Update();

        // save the depth img as a png
        VTK_CREATE(vtkPNGWriter, writer);
        writer->SetFileName ("outputDepth.png");
        writer->SetInputConnection (scalerDepth->GetOutputPort ());
        writer->Write ();
        std::cout << "Wrote image Depth." << std::endl;



        // read RGB image from vtk Window
        VTK_CREATE (vtkWindowToImageFilter, filterRGB);
        filterRGB->SetInput (rwi->GetRenderWindow());
        filterRGB->SetMagnification( 1 );
        filterRGB->SetInputBufferTypeToRGBA();
        filterRGB->ReadFrontBufferOff();
        filterRGB->Update ();

        VTK_CREATE(vtkImageData, imrgb);
        imrgb = filterDepth->GetOutput();
        dimsdepth[3];
        imrgb->GetDimensions(dimsdepth);

        VTK_CREATE(vtkImageShiftScale, scalerRGB);
        // scaler->SetOutputScalarTypeToUnsignedShort();
        // scaler->SetInputConnection(imdepth->GetProducerPort());
        // scaler->SetShift(-1.0f * image->GetScalarRange()[0]); // brings the lower bound to 0
        // scaler->SetScale(newRange/oldRange);
        scalerRGB->SetOutputScalarTypeToUnsignedChar(); 
        scalerRGB->SetInputConnection(filterRGB->GetOutputPort()); 
        scalerRGB->SetShift(0); 
        scalerRGB->SetScale(-255);
        scalerRGB->Update();

        // save the depth img as a png
        VTK_CREATE(vtkPNGWriter, writerRGB);
        writerRGB->SetFileName ("outputRGB.png");
        writerRGB->SetInputConnection (scalerRGB->GetOutputPort ());
        writerRGB->Write ();
        std::cout << "Wrote image RGB." << std::endl;




        }
 
      // Handle a "normal" key
      if(key == "a")
        {
        std::cout << "The a key was pressed." << std::endl;
        }
 
      // Forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
 
};
vtkStandardNewMacro(KeyPressInteractorStyle);

int main ( int argc, char *argv[] )
{
  if(argc != 2)
    {
    std::cout << "Usage: " << argv[0] << "  Filename(.ply)" << std::endl;
    return EXIT_FAILURE;
    }
 
  std::string inputFilename = argv[1];
  VTK_CREATE(vtkPLYReader, reader);
  reader->SetFileName (inputFilename.c_str());
  reader->Update ();

  // create depth sort algo
  VTK_CREATE(vtkDepthSortPolyData, sort);
  sort->SetInput(reader->GetOutput ());

  // init stuff for rendering
  VTK_CREATE(vtkPolyDataMapper, mapper);
  VTK_CREATE(vtkActor, actor);
  VTK_CREATE(vtkRenderer, rend);
  VTK_CREATE(vtkRenderWindow, rw);
  VTK_CREATE(vtkRenderWindowInteractor, inte);
  VTK_CREATE(KeyPressInteractorStyle, style);

  VTK_CREATE(vtkCamera, cam);
  sort->SetDirectionToBackToFront (); // camera direction
  sort->SetCamera (cam); // set camera or runtime warning
  sort->SortScalarsOn ();
  sort->Update ();

  mapper->SetScalarVisibility(true);

  // limit max scalar (nb Color)
  mapper->SetScalarRange (0, sort->GetOutput ()->GetNumberOfCells ());
  mapper->SetInputConnection(sort->GetOutputPort ());
  mapper->Update ();

  actor->SetMapper(mapper);
  // actor->RotateY (95); // transform with a rotation to see depth
  // actor->GetProperty()->SetColor(1, 0, 0);
  actor->GetProperty()->SetInterpolationToFlat();
  actor->GetProperty()->LightingOff();
  actor->GetProperty()->ShadingOff();
  sort->SetProp3D (actor); // set the actor to the algo

  rend->SetActiveCamera (cam);
  rw->AddRenderer(rend);
  rend->AddActor(actor);
  inte->SetRenderWindow (rw);
  inte->SetInteractorStyle(style);
  style->SetCurrentRenderer(rend);

  inte->Initialize ();


  // start rendering for visualization
  rw->Render ();



  inte->Start ();
  return 0;
}