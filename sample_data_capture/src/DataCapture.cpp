// DataCapture.cpp (Stephanie Cockrell) This program takes in data from the kinect (rgb image, depth image, point cloud) and writes to file

// Include NITE
#include "/home/stephanie/drivers/nite/build/Nite-1.3.1.5/Include/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/CvBridge.h>

//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>

using namespace std;

// This macro checks the return code that all OpenNI calls make
// and throws an error if the return code is an error. Use this
// after all calls to OpenNI objects. Great for debugging.
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

bool savedTheImage=false; // ros needs to continue spinOnce until the image is saved

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("hey homedawg, you're in imageCallback\n");
	sensor_msgs::CvBridge bridge; // use CvBridge to convert ros msgs to opencv
	IplImage* image;
	try
	{
		image=bridge.imgMsgToCv(msg, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	cvSaveImage("rgb_output.jpg",image);
	savedTheImage=true;
}

int main(int argc, char* argv[]){
	// These variables are for receiving kinect depth map data:
	// Keep track of the return code of all OpenNI calls
	XnStatus nRetVal = XN_STATUS_OK;
	// context is the object that holds most of the things related to OpenNI
	xn::Context context;
	// The DepthGenerator generates a depth map that we can then use to do 
	// cool stuff with. Other interesting generators are gesture generators
	// and hand generators.
	xn::DepthGenerator depth;
	
	// Initialize context object
	nRetVal = context.Init();
	CHECK_RC(nRetVal, "Initialize context");
	// Create the depth object
	nRetVal = depth.Create(context);
	CHECK_RC(nRetVal, "Create Depth");
	
	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "Start Generating All Data");

	ros::init(argc, argv, "data_capture");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color", 1, imageCallback);
	//image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

	ros::Rate r(15); //run at 15 Hz

	int count_iterations=0;

	int x_res=XN_VGA_X_RES; // should be 640- the width of the depth matrix returned by the kinect
	int y_res=XN_VGA_Y_RES;	// should be 480

	const XnDepthPixel* pDepthMap;
	XnPoint3D realWorld[x_res*y_res];
	// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];

	// These are the files where the depth data is stored
	ofstream myfile_depthmap; 
	myfile_depthmap.open ("depthmap.txt");	
	ofstream myfile_realworld; 
	myfile_realworld.open ("realworld.txt");

	// Now the part where you actually read in the data (so if I was gonna make a while loop, it would start here):
	// Wait for new data to be available
	nRetVal = context.WaitOneUpdateAll(depth);
	CHECK_RC(nRetVal, "Updating depth");
	// Get the new depth map
	pDepthMap = depth.GetDepthMap();
	// notice that I'm not using ros to subscribe to a depth map- I'm using this GetDepthMap method instead
	// there's a very good reason for this, which is: that's just how the code I copied from the intarwebz was
	// if you wanna use ros to subscribe instead, knock yourself out
	// I suspect the topic to subscribe to is camera/depth/image_rect

	for (int y=0; y<XN_VGA_Y_RES; y++){
		for(int x=0;x<XN_VGA_X_RES;x++){
		        pointList[y * XN_VGA_X_RES + x ].X =x;
		        pointList[y * XN_VGA_X_RES + x ].Y =y;
		        pointList[y * XN_VGA_X_RES + x ].Z = (short) pDepthMap[y *XN_VGA_X_RES + x];
			myfile_depthmap << pDepthMap[y *XN_VGA_X_RES + x] ;
			if(x<XN_VGA_X_RES-1){
				myfile_depthmap << ", ";
			}
		}
		myfile_depthmap << "\n" ;
	} 

	// this function is super-useful and badly documented so just trust me
	depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

	// write-to-file all the real-world (x,y,z) coordinates
	// by the way, the units are mm
	for(int i=0; i<x_res; i++){ 
		for(int j=0; j<y_res; j++){
			myfile_realworld << realWorld[j*x_res + i].X << ", ";
			myfile_realworld << realWorld[j*x_res + i].Y << ", ";
			myfile_realworld << realWorld[j*x_res + i].Z << "\n";
		}
	
	}

	// now use ros to get the rgb image
	while(!savedTheImage){
		ros::spinOnce();
	}

	myfile_depthmap.close();
	myfile_realworld.close();

	// Clean-up
	context.Shutdown();
	return 0;
}
