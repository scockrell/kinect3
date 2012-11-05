// This program just outputs the raw data from xn stuff
// so I can see which dimension is x, y, and z wrt the kinect
// creates a file xn_raw.txt
// Stephanie Cockrell
// September 6, 2012

// possibly useful: http://www.ros.org/doc/api/nao_openni/html/namespacexn.html
// Include OpenNI
//#include </home/mobilerobots/kinect/OpenNI/Include/XnCppWrapper.h>
//#include </home/stephanie/code/dev_stacks/kinect/OpenNI/Include/XnCppWrapper.h>
// Include NITE
//#include "/home/mobilerobots/kinect/NITE/Nite-1.4.0.5/Include/XnVNite.h"
#include "/home/stephanie/code/dev_stacks/kinect/NITE/Include/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <math.h>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

using namespace std;
//using namespace sensor_msgs;



// This macro checks the return code that all OpenNI calls make
// and throws an error if the return code is an error. Use this
// after all calls to OpenNI objects. Great for debugging.
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

#define _TCHAR char
#define _tmain main

int _tmain(int argc, _TCHAR* argv[]){
	//
	// Variables

	// Keep track of the return code of all OpenNI calls
	XnStatus nRetVal = XN_STATUS_OK;
	// context is the object that holds most of the things related to OpenNI
	xn::Context context;
	// The DepthGenerator generates a depth map that we can then use to do 
	// cool stuff with. Other interesting generators are gesture generators
	// and hand generators.
	xn::DepthGenerator depth;

	//
	// Initialization
	
	// Initialize context object
	nRetVal = context.Init();
	CHECK_RC(nRetVal, "Initialize context");
	// Create the depth object
	nRetVal = depth.Create(context);
	CHECK_RC(nRetVal, "Create Depth");
	
	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "Start Generating All Data");

	int total_iterations=10;
	int count_iterations=0;

	int real_x_dim=640;
	int real_y_dim=480;

	int x_res=XN_VGA_X_RES;
	int y_res=XN_VGA_Y_RES;	

	const XnDepthPixel* pDepthMap;
	XnPoint3D realWorld[real_x_dim*real_y_dim];
	// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];

	// Main loop
	while(count_iterations<1)
	//while (ros::ok()) //(n.ok()) // hey i don't know man, I just commented that out because of this http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	{
		// Wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		CHECK_RC(nRetVal, "Updating depth");
		// Get the new depth map
		pDepthMap = depth.GetDepthMap();

		for (int y=0; y<XN_VGA_Y_RES; y++){
			for(int x=0;x<XN_VGA_X_RES;x++){
			        pointList[y * XN_VGA_X_RES + x ].X =x;
			        pointList[y * XN_VGA_X_RES + x ].Y =y;
			        pointList[y * XN_VGA_X_RES + x ].Z = (short) pDepthMap[y *XN_VGA_X_RES + x];
			}
		} 



		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth. OR IS IT?

		ofstream myfile;
		myfile.open ("xn_raw.txt");

		for(int i=0; i<x_res; i++){ 
			for(int j=0; j<y_res; j++){
				myfile << realWorld[j*x_res + i].X << "," ;
				myfile << realWorld[j*x_res + i].Y << "," ;
				myfile << realWorld[j*x_res + i].Z << "\n" ;

			}			
		}

		myfile.close();
		count_iterations++;
	}


	// Clean-up
	context.Shutdown();
	return 0;
}
