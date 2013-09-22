#undef MAX
#undef MIN
#undef max
#define NOMINMAX
#include "openni_pcl.h"
#include "openni_init.h"

int main()
{
	std::cout<<"OpenNI2 Grabber\n";
	openni2_grabber kinect;
	/*Change these functions to display color/depth or both streams
	Refer to documentation for various functions that can be used for these purposes*/
	kinect.createDualStreams();
	kinect.startPointCloudChangeVisualizer(); //Displays Point cloud using PCL Visualizer
	//kinect.startPCLRGBCloudVisualizer();   
	ReadLastCharOfLine();
	return 0;
}