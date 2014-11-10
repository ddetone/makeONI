/*****************************************************************************
*                                                                            *
*  OpenNI 1.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include <iostream>
#include <string>
#include <vector>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../../../Data/SamplesConfig.xml"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace cv;
using namespace std;
using namespace xn;
using namespace boost::filesystem;

void transformDepthMD(DepthMetaData& depthMD)
{
	DepthMap& depthMap = depthMD.WritableDepthMap();
	for (XnUInt32 y = 0; y < depthMap.YRes(); y++)
	{
		for (XnUInt32 x = 0; x < depthMap.XRes(); x++)
		{
			//Punch vertical cut lines in the depth image
			if ((x % 4) == 0)
			{
				depthMap(x, y) = 0;
			}
		}
	}
}

int main(int argc, char* argv[])
{
	XnStatus nRetVal = XN_STATUS_OK;
	nRetVal = xnLogInitFromXmlFile(SAMPLE_XML_PATH);
	if (nRetVal != XN_STATUS_OK)
	{
		//printf("Log couldn't be opened: %s. Running without log", xnGetStatusString(nRetVal));
	}

	if (argc < 3)
	{
		printf("usage: %s <inputFile> <outputFile>\n", argv[0]);
		return -1;
	}

	const char* strInputFile = argv[1];
	const char* strOutputFile = argv[2];

	Context context;
	nRetVal = context.Init();
	CHECK_RC(nRetVal, "Init");

	// open input file
	Player player;
	nRetVal = context.OpenFileRecording(strInputFile, player);
	CHECK_RC(nRetVal, "Open input file");

	// Get depth node from recording
	DepthGenerator depth;
	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
	CHECK_RC(nRetVal, "Find depth generator");

	// Create mock node based on depth node from recording
	MockDepthGenerator mockDepth;
	nRetVal = mockDepth.CreateBasedOn(depth);
	CHECK_RC(nRetVal, "Create mock depth node");

	// create recorder
	Recorder recorder;

	nRetVal = recorder.Create(context);
	CHECK_RC(nRetVal, "Create recorder");

	nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, strOutputFile);
	CHECK_RC(nRetVal, "Set recorder destination file");

	// add depth node to recorder
	nRetVal = recorder.AddNodeToRecording(mockDepth);
	CHECK_RC(nRetVal, "Add node to recording");

	nRetVal = player.SetRepeat(FALSE);
	XN_IS_STATUS_OK(nRetVal);

	XnUInt32 nNumFrames = 0;

	nRetVal = player.GetNumFrames(depth.GetName(), nNumFrames);
	CHECK_RC(nRetVal, "Get player number of frames");


	// Create array with all filenames
  typedef vector<path> vec;             // store paths,
  vec v;                                // so we can sort them later
  path p ("/home/ddetone/code/makeONI/Data/raw/basement_0001a/");
  try
  {
    if (exists(p))    // does p actually exist?
    {
      if (is_directory(p))      // is p a directory?
      {
        //cout << p << " is a directory containing:\n";
        copy(directory_iterator(p), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end());             // sort, since directory iteration
      }
      else
        cout << p << " exists, but is not a directory\n";
    }
    else
      cout << p << " does not exist\n";
  }
  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

	// Create vector with only pgm strings
	vector<string> img_paths;	
	for(vec::iterator it = v.begin(); it != v.end(); ++it) 
	{
		if (it->extension().string() == ".pgm")
		{
			string current_file = it->string();
			cout << current_file << endl;
			img_paths.push_back(current_file);
    }
	}

	//Mat image = imread(img_paths[0], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
	Mat image = imread("home/ddetone/code/makeONI/Data/raw/basement_0001a/test_number.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
  if(! image.data )                              // Check for invalid input
  {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
  }
  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", image );                   // Show our image inside it.

  waitKey(0); 

		//imgs.push_back(

  //directory_iterator end_itr;
 
  //// cycle through the directory
  //for (directory_iterator itr(p); itr != end_itr; ++itr)
  //{
  //    // If it's not a directory, list it. If you want to list directories too, just remove this check.
  //    if (is_regular_file(itr->path())) {
  //        // assign current file name to current_file and echo it out to the console.
	//				string ext = itr->path().extension().string();
	//				if (ext == ".pgm")
	//				{
  //        	string current_file = itr->path().string();
  //        	cout << current_file << endl;
	//					fnames.push_back(current_file);
	//				}
  //    }
  //}

	DepthMetaData depthMD;
	while ((nRetVal = depth.WaitAndUpdateData()) != XN_STATUS_EOF)
	{
		CHECK_RC(nRetVal, "Read next frame");
		
		// Get depth meta data
		depth.GetMetaData(depthMD);

		//-----------------------------------------------//
		// Transform depth! This is the interesting part //
		//-----------------------------------------------//
		
		/* Enable the depth data to be modified. This is done implicitly by depthMD.WritableDepthMap(),
		   but we're calling it just to be clear. */
		nRetVal = depthMD.MakeDataWritable();
		CHECK_RC(nRetVal, "Make depth data writable");

		transformDepthMD(depthMD);

		// Pass the transformed data to the mock depth generator
		nRetVal = mockDepth.SetData(depthMD);
		CHECK_RC(nRetVal, "Set mock node new data");

		/* We need to call recorder.Record explicitly because we're not using WaitAndUpdateAll(). */
		nRetVal = recorder.Record();
		CHECK_RC(nRetVal, "Record");

		printf("Recorded: frame %u out of %u\r", depthMD.FrameID(), nNumFrames);

		if (depthMD.FrameID() == 100)
			break;
	}

	printf("\n");

	return 0;
}
