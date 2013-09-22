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
	kinect.createDualStreams();
	kinect.startPointCloudChangeVisualizer();
	//kinect.startPCLRGBCloudVisualizer();
	ReadLastCharOfLine();
	return 0;
}