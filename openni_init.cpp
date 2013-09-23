#include "openni_init.h"

openni2_grabber::openni2_grabber()/* : viewer("Kinect Depth Map"),imageViewer("Kinect RGB Stream")*/
{
	printver();
    status = OpenNI::initialize();
    std::cout<<"Initializing..\n";
    if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}

    OpenNI::enumerateDevices(&devinfo);
    int totaldevices =devinfo.getSize();
    if(totaldevices == 0)
    {
        std::cout<<"No devices connected..\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
    }
    else
    {
        std::cout<<totaldevices<<" device(s) connected\n";
        for(int i=0;i<totaldevices;i++)
        {
            openni::DeviceInfo device = devinfo[i];
            printf("%d. %s->%s (VID: %d | PID: %d) is connected "
            " at %s\r\n",
            i,
            device.getVendor(),
            device.getName(),
            device.getUsbVendorId(),
            device.getUsbProductId(),
            device.getUri());
        }
    }
	int dev_num = 0; //since only kinect
    std::cout<<"Selected device: "<<devinfo[dev_num].getName()<<std::endl;
    std::cout<<"Opening "<<devinfo[dev_num].getName()<<"...\n";
    
    status = device.open(devinfo[dev_num].getUri());
    std::cout<<"Done..\n";

	isDepthModeSet = false;
	isColorModeSet = false;
	isIRModeSet = false;
	isDepthStream = false;
	isColorStream = false;
	isIRStream = false;
	isSyncReg = false;

	this->defaultDepthMode.setFps(30);
	this->defaultDepthMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	this->defaultDepthMode.setResolution(640,480);

	this->defaultColorMode.setFps(30);
	this->defaultColorMode.setPixelFormat(PIXEL_FORMAT_RGB888);
	this->defaultColorMode.setResolution(640,480);

	this->defaultIRMode.setFps(30);
	this->defaultIRMode.setPixelFormat(PIXEL_FORMAT_GRAY16);
	this->defaultIRMode.setResolution(640,480);

};

void openni2_grabber::setDepthMode()
{
	status = depthSensor.setVideoMode(defaultDepthMode);
	isDepthModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::setColorMode()
{
	status = colorSensor.setVideoMode(defaultColorMode);
	isColorModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::setIRMode()
{
	status = IRSensor.setVideoMode(defaultIRMode);
	isIRModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::setDepthMode(VideoMode mode)
{
	depthMode = mode;
	status = depthSensor.setVideoMode(depthMode);
	isDepthModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::setColorMode(VideoMode mode)
{
	colorMode = mode;
	status = colorSensor.setVideoMode(colorMode);
	isColorModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::setIRMode(VideoMode mode)
{
	IRMode = mode;
	status = IRSensor.setVideoMode(mode);
	isIRModeSet = true;
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
};

void openni2_grabber::createDepthStream()
{
	std::cout<<"Requesting depth stream..\n";
	if(device.hasSensor(SENSOR_DEPTH))
    {
        std::cout<<"Note: Depth sensor available\n";
    }
    else
    {
        std::cout<<"Depth sensor not available..\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
    }
	std::cout<<"Creating Depth stream..\n";
	status = depthSensor.create(device,SENSOR_DEPTH);
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	std::cout<<"Done..\n";
	std::cout<<"Setting videomode..\n";
	if(!isDepthModeSet) this->setDepthMode();
	std::cout<<"Starting the Depth stream..\n";
	status = depthSensor.start();
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	isDepthStream = true;
	std::cout<<"Done..\n";
};

void openni2_grabber::createColorStream()
{
	std::cout<<"Requesting color stream..\n";
	if(IRSensor.isValid())
	{
		std::cout<<"Warning! cannot use IR and color streams at the same time..\nStutting down IR stream..\n";
		IRSensor.stop();
		IRSensor.destroy();
		isIRStream = false;
	}
	if(device.hasSensor(SENSOR_COLOR))
    {
        std::cout<<"Color sensor available..\n";
    }
    else
    {
        std::cout<<"Color sensor not available..\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
    }
	std::cout<<"Creating Color stream..\n";
	status = colorSensor.create(device,SENSOR_COLOR);
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	std::cout<<"Done..\n";
	std::cout<<"Setting videomode..\n";
	if(!isDepthModeSet) this->setColorMode();
	std::cout<<"Starting the Color stream..\n";
	status = colorSensor.start();
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	 isColorStream = true;
	 std::cout<<"Done..\n";
};

void openni2_grabber::createIRStream()
{
	std::cout<<"Requesting IR stream..\n";
	if(colorSensor.isValid())
	{
		std::cout<<"Warning! cannot use IR and color streams at the same time..\nStutting down Color stream..\n";
		colorSensor.stop();
		colorSensor.destroy();
		isColorStream = false;
	}
	if(!device.hasSensor(SENSOR_IR))
	{
		std::cout<<"IR sensor not available..\n";
		exit(EXIT_FAILURE);
	}
	std::cout<<"Creating IR stream..\n";
	status = IRSensor.create(device,SENSOR_IR);
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	std::cout<<"Done..\n";
	std::cout<<"Setting videomode..\n";
	if(!isIRModeSet) this->setIRMode();
	std::cout<<"Starting IR stream..\n";
	status = IRSensor.start();
	if(!handlestatus(status))
	{
		std::cout<<status;
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	isIRStream = true;
	std::cout<<"Done..\n";
};

void openni2_grabber::createDualStreams()
{
	if(isColorStream && isDepthStream)
	{
		std::cout<<"Both the streams are already created..\n";
	}
	else if(isColorStream)
	{
		std::cout<<"Color Stream already enabled. Creating the depth Stream..\n";
		this->createDepthStream();
	}
	else if(isDepthStream)
	{
		std::cout<<"Depth Stream already enabled. Creating the color Stream..\n";
		this->createColorStream();
	}
	else
	{
		this->createColorStream();
		this->createDepthStream();
	}
	std::cout<<"Enabling Depth - Color stream Sync..\n";
	status = device.setDepthColorSyncEnabled(true);
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	std::cout<<"Enabling Depth to Image registration mode..\n";
	status = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	if(!handlestatus(status))
	{
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	std::cout<<"Done..\n";
	isSyncReg = true;
};

VideoFrameRef openni2_grabber::getDepthFrame()
{
	if(isDepthStream)
	{
		status = depthSensor.readFrame(&depthFrame);
		if(!depthFrame.isValid()) std::cout<<"No valid depth data read..Check Device connection..\n";
		if(!handlestatus(status))
		{
			ReadLastCharOfLine();
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		std::cout<<"No depth stream found..Please start the depth stream...\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	return depthFrame;
};

VideoFrameRef openni2_grabber::getColorFrame()
{
	if(isColorStream)
	{
		status = colorSensor.readFrame(&colorFrame);
		if(!colorFrame.isValid()) std::cout<<"No valid color data read..Check Device connection..\n";
		if(!handlestatus(status))
		{
			ReadLastCharOfLine();
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		std::cout<<"No color stream found..Please start the color stream...\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	return colorFrame;
};

VideoFrameRef openni2_grabber::getIRFrame()
{
	if(isIRStream)
	{
		status = IRSensor.readFrame(&IRFrame);
		if(!IRFrame.isValid()) std::cout<<"No valid  IR data read...check device connection..\n";
		if(!handlestatus(status))
		{
			ReadLastCharOfLine();
			exit(EXIT_FAILURE);
		}
	}
	else
	{
		std::cout<<"No color stream found..Please start the IR stream...\n";
		ReadLastCharOfLine();
		exit(EXIT_FAILURE);
	}
	return IRFrame;
};

pcl::PointCloud<pcl::PointXYZ> openni2_grabber::getPointCloud()
{
		return cloud_xyz(getDepthFrame(),depthSensor);
};

openni2_grabber::~openni2_grabber()
{
	if(isColorStream)
	{
		colorSensor.stop();
		colorSensor.destroy();
	}
	if(isDepthStream)
	{
		depthSensor.stop();
		depthSensor.destroy();
	}
	if(isIRStream)
	{
		IRSensor.stop();
		IRSensor.destroy();
	}
	device.close();
	OpenNI::shutdown();
}


void openni2_grabber::startPCLCloudVisualizer()
{
	pcl::visualization::PCLVisualizer viewer("Kinect Depth Map");
	std::cout<<"Starting the PCL Cloud Visualizer..\n";
	viewer.setBackgroundColor(0,0,0);
	Eigen::Matrix4f transmat;
	transmat<<-1,0,0,0,
			   0,1,0,0,
			   0,0,-1,-2000,
			   0,0,0,1;
	pcl::PointCloud<pcl::PointXYZ> transformCloud;
	pcl::transformPointCloud(getPointCloud(),transformCloud,transmat);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZ>(transformCloud));
	viewer.addPointCloud<pcl::PointXYZ>(depthCloud,"depthmap");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"depthmap");
	viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
	viewer.setWindowBorders(true);

	while(!viewer.wasStopped())
	{
		viewer.spinOnce(30);
		pcl::transformPointCloud(getPointCloud(),transformCloud,transmat);
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZ>(/*cloud_xyz(depthFrame,depthSensor)*/transformCloud));
		viewer.updatePointCloud(depthCloud,"depthmap");
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
	}
};

void openni2_grabber::startPCLImageVisualizer()
{
	pcl::visualization::ImageViewer imageViewer;
	imageViewer.setWindowTitle("Kinect RGB Stream");
	std::cout<<"Starting PCL Image Visualizer..\n";
	imageViewer.addRGBImage((unsigned char*)getColorFrame().getData(),colorFrame.getWidth(),colorFrame.getHeight(),"rgbImage");

	while(!imageViewer.wasStopped())
	{
		imageViewer.spinOnce(30);
		imageViewer.removeLayer("rgbImage");
		imageViewer.addLayer("rgbImage",colorFrame.getWidth(),colorFrame.getHeight());
		imageViewer.addRGBImage((unsigned char*)getColorFrame().getData(),colorFrame.getWidth(),colorFrame.getHeight(),"rgbImage");
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
	}
};

void openni2_grabber::startPCLIRVisualizer()
{
	pcl::visualization::ImageViewer IRViewer;
	IRViewer.setWindowTitle("Kinect IR Stream");
	std::cout<<"Starting PCl Visualizer of IR stream..\n";
	IRViewer.addMonoImage((unsigned char*)getIRFrame().getData(),IRFrame.getWidth(),IRFrame.getHeight(),"IRImage");
	while(!IRViewer.wasStopped())
	{
		IRViewer.spinOnce(30);
		IRViewer.removeLayer("IRImage");
		IRViewer.addLayer("IRImage",IRFrame.getWidth(),IRFrame.getHeight());
		IRViewer.addMonoImage((unsigned char*)getIRFrame().getData(),IRFrame.getWidth(),IRFrame.getHeight(),"IRImage");
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
	}
};

pcl::PointCloud<pcl::PointXYZRGB> openni2_grabber::getRGBPointCloud()
{
	if(!isSyncReg) std::cout<<"Depth to color Registration/ Depth-Color sync is not enabled..\n";
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	float px,py,pz;

	DepthPixel *depthPixels,dp;
	RGB888Pixel *colorPixels,cp;

	depthPixels = (DepthPixel*)((char*)getDepthFrame().getData());
	colorPixels = (RGB888Pixel*)((char*)getColorFrame().getData());

	cloud.width = depthFrame.getWidth();
	cloud.height = depthFrame.getHeight();
	cloud.resize(cloud.width * cloud.height);

	for(int y=0;y<depthFrame.getHeight();y++)
	{
		for(int x=0;x<depthFrame.getWidth();x++)
		{
			dp = *depthPixels;
			cp = *colorPixels;
			status = CoordinateConverter::convertDepthToWorld(depthSensor,x,y,dp,&px,&py,&pz);
			if(!handlestatus(status))
			{
				ReadLastCharOfLine();
				exit(EXIT_FAILURE);
			}
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].x = px;
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].y = py;
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].z = pz;
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].r = cp.r;
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].g = cp.g;
			cloud.points[x + (depthFrame.getStrideInBytes() * y)/2].b = cp.b;

			depthPixels++;
			colorPixels++;
		}
	}
	return cloud;
};

void openni2_grabber::startPCLRGBCloudVisualizer()
{
	pcl::visualization::PCLVisualizer viewer("Kinect RGB Cloud");
	std::cout<<"Starting the PCL RGB Cloud Visualizer..\n";
	viewer.setBackgroundColor(0,0,0);
	Eigen::Matrix4f transmat;
	transmat<<-1,0,0,0,
			   0,1,0,0,
			   0,0,-1,-2000,
			   0,0,0,1;
	pcl::PointCloud<pcl::PointXYZRGB> transformCloud;
	pcl::transformPointCloud(getRGBPointCloud(),transformCloud,transmat);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZRGB>(transformCloud));
	viewer.addPointCloud<pcl::PointXYZRGB>(depthCloud,"RGBdepthmap");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RGBdepthmap");
	viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
	viewer.setWindowBorders(true);

	while(!viewer.wasStopped())
	{
		viewer.spinOnce(30);
		pcl::transformPointCloud(getRGBPointCloud(),transformCloud,transmat);
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZRGB>(/*cloud_xyz(depthFrame,depthSensor)*/transformCloud));
		viewer.updatePointCloud(depthCloud,"RGBdepthmap");
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
	}
};

void openni2_grabber::startPointCloudChangeVisualizer()
{
	float resolution = 32.0f;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);
	pcl::PointCloud<pcl::PointXYZRGB> cloud = getRGBPointCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB> (cloud));
	
	octree.setInputCloud(cloudPtr);
	octree.addPointsFromInputCloud();

	octree.switchBuffers();

	pcl::visualization::PCLVisualizer viewer("Kinect RGB CLoud Change Visualizer");
	std::cout<<"Starting the PCL RGB Cloud Change detector Visualizer..\n";
	viewer.setBackgroundColor(0,0,0);
	Eigen::Matrix4f transmat;
	transmat<<-1,0,0,0,
			   0,1,0,0,
			   0,0,-1,-2000,
			   0,0,0,1;
	pcl::PointCloud<pcl::PointXYZRGB> transformCloud;
	pcl::transformPointCloud(cloud,transformCloud,transmat);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZRGB>(transformCloud));
	viewer.addPointCloud<pcl::PointXYZRGB>(depthCloud,"RGBdepthmap_ChangeDectector");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"RGBdepthmap_ChangeDectector");
	viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
	viewer.setWindowBorders(true);

	std::vector<int> indices;

	while(!viewer.wasStopped())
	{

		viewer.spinOnce(30);
		pcl::PointCloud<pcl::PointXYZRGB> cloudChanged = getRGBPointCloud();
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB> (cloudChanged));
		octree.setInputCloud(cloudPtr);
		octree.addPointsFromInputCloud();
		octree.getPointIndicesFromNewVoxels(indices);
		//std::cout<<"# of changed points: "<<indices.size()<<std::endl;

		for(int i=0;i<indices.size();i++)
		{
			cloudChanged.points[indices[i]].r = 255;
			cloudChanged.points[indices[i]].g = 0;
			cloudChanged.points[indices[i]].b = 0;
		}

		pcl::transformPointCloud(cloudChanged,transformCloud,transmat);
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr depthCloud(new pcl::PointCloud<pcl::PointXYZRGB>(/*cloud_xyz(depthFrame,depthSensor)*/transformCloud));
		viewer.updatePointCloud(depthCloud,"RGBdepthmap_ChangeDectector");
		boost::this_thread::sleep(boost::posix_time::milliseconds(30));
		octree.switchBuffers();
	}
};