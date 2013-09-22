#undef MAX
#undef MIN
#undef max
#define NOMINMAX
#include "openni_pcl.h"
#include "openni_init.h"

int main()
{
	std::cout<<"OpenNI2 Grabber\n";
	openni2_grabber kinect;    //Initialize OpenNI2 modules
	/*Change these functions to display color/depth or both streams
	Refer to documentation for various functions that can be used for these purposes*/   
	kinect.createDualStreams();  //create Color and Depth Streams
	//kinect.startPointCloudChangeVisualizer();
	kinect.startPCLRGBCloudVisualizer(); //start visualizing RGB poit cloud data
	ReadLastCharOfLine();
	return 0;
}
//The destructor takes care of closing the device and shutting down OpenNI modules
