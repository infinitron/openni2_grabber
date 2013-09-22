#ifndef _OPENNI_INIT_H
#define _OPENNI_INIT_H

#include "openni_pcl.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/octree/octree.h>
#include <vector>

class openni2_grabber
{
public:
	openni2_grabber();
	Status status;
	void createDepthStream();
	void createColorStream();
	void setDepthMode(VideoMode);
	void setColorMode(VideoMode);
	void setDepthMode();
	void setColorMode();
	void createDualStreams();
	VideoFrameRef getDepthFrame();
	VideoFrameRef getColorFrame();
	pcl::PointCloud<pcl::PointXYZ> getPointCloud();
	pcl::PointCloud<pcl::PointXYZRGB> getRGBPointCloud();
	void startPCLCloudVisualizer();
	void startPCLImageVisualizer();
	void startPCLRGBCloudVisualizer();
	void startPointCloudChangeVisualizer();
	~openni2_grabber();
	
protected:
	Array<DeviceInfo> devinfo;
	bool isDepthModeSet;
	bool isColorModeSet;
	bool isDepthStream;
	bool isColorStream;
	bool isSyncReg;
	Device device;

	VideoStream depthSensor;
	VideoStream colorSensor;
	VideoMode depthMode;
	VideoMode colorMode;
	VideoFrameRef depthFrame;
	VideoFrameRef colorFrame;
	VideoMode defaultDepthMode;
	VideoMode defaultColorMode;
	//pcl::visualization::PCLVisualizer viewer;
	//pcl::visualization::ImageViewer imageViewer;
};

#endif
