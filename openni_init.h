/*! \file openni_init.h*/
#ifndef _OPENNI_INIT_H
#define _OPENNI_INIT_H

#include "openni_pcl.h"
#include <pcl/visualization/image_viewer.h>
#include <pcl/octree/octree.h>
#include <vector>

//! OpenNI2 Grabber class
/*! This class contains the functions to use the data from OpenNI supported devices such as Microsoft Kinect and ASUS Xtion Pro.
 It uses various functions from the Point Cloud Library (PCL) for better visualization and also to avoid working with low level commands from OpenGL.
*/
class openni2_grabber
{
public:
	//!The default Constructor
	/*!The constructor automatically initializes and loads the drivers necessary to acquire data from the supported
	devices. By default it loads the first device that appears in the list of connected devices.
	*/
	openni2_grabber();
	//!A global variable used by the member functions to indicate the status of the calls to OpenNI
	Status status;
	//!Depth stream initialization
	/*!This function initlializes the depth stream from the device. If no video mode is supplied it uses the defafult
	VideoMode object openni2_grabber::defaultDepthMode. VideoMode can also be set manually by calling the function setDepthMode(VideoMode mode).
	This fuction must be called before starting data aquisition from the depth sensor*/ 
	void createDepthStream();
	//!Color Stream Initialization
	/*!This function initializes the color stream from the device. It uses the default Videomode object defaultColorMode if
	no video mode is set. VideoMode can also be set manually by calling setColorMode(VideoMode mode).
	This function must be called before starting data aquisition from the color sensor.
	*/
	void createColorStream();
	//!IR stream initialization
	/*!This function initializes IR stream from the device. It uses the default videomode if no videoMode is
	set. VideoMode can also be set manually using setIRMode(VideoMode mode). This function must be called before
	startig data acquisition from the IR sensor.
	*/
	void createIRStream();
	//!This function sets the VideoMode to be set to the Depth Stream.
	/*!
	\param mode An object of type openni::VideoMode
	*/
	void setDepthMode(VideoMode mode);
	//!This function sets the VideoMode to be set to the Color Stream.
	/*!
	\param mode An object of type openni::VideoMode
	*/
	void setColorMode(VideoMode mode);
	//!This function sets the VideoMode for the IR stream
	/*!
	\param mode An object of type openni::VideoMode
	*/
	void setIRMode(VideoMode mode);
	//!Initialize Depth and Color Streams
	/*!It also enables Depth to Color Synchronization. The Image registration mode will be set to 
	<B>openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR</B>.
	*/
	void createDualStreams();
	//!Returns VideoFrameRef object for DepthStream
	/*!It returns VideoFrameRef object contains the depth data from the device. createDepthStream() must be called before calling this function.*/
	VideoFrameRef getDepthFrame();
	//!Returns VideoFrameRef object which contains the color data from the device.
	/*!createColorStream must be called before starting data acquisition*/
	VideoFrameRef getColorFrame();
	//!Returns VideoFrameRef object for IR stream
	/*!createIRStream() must be called before starting IR data acqisition*/
	VideoFrameRef getIRFrame();
	//!Get point cloud data from the device
	/*!This funtion returns the point cloud generated using the depth data from the device. createDepthStream() must be called before using this function
	The cloud will be of the type <B>pcl::PointCloud<pcl::PointXYZ></B>.*/
	pcl::PointCloud<pcl::PointXYZ> getPointCloud();
	//!Returns the RGBXYZ PointCloud
	/*!This function returns the RGB PointCloud generated using the Depth and Color streams from the device. Either createDepthStream() and createColorStream() both or
	create createDualStreams() must be called before using this funtion. The data returned will be of the type <B>pcl::PointCloud<pcl::PointXYZRGB></B>. */
	pcl::PointCloud<pcl::PointXYZRGB> getRGBPointCloud();
	//!Displays the PointCloud from the device
	/*!This function displays the point cloud using the DepthStream of the device. createDepthStream() must be called before displaying the point cloud. Point size of the cloud
	is 1. The parameter of the coordinate system is 1.0. The PointCloud is rotated 180 degrees about the Y axis and translated -2000 units along the Z axis for better visualization*/
	void startPCLCloudVisualizer();
	//!Displays the RGB Stream fron the device
	/*!This fuction displays the RGB Image stream from the device. createColorStream() must be called before displaying the stream. */
	void startPCLImageVisualizer();
	//!Displays the IR stream from the device
	/*!This function displays the IR stream from the device. createIRStream() must be called before starting
	visualization*/
	void startPCLIRVisualizer();
	//!Displays the coloured point cloud
	/*!This function displays the coloured point cloud using the data from the device.Either createDepthStream() and createColorStream() both or
	create createDualStreams() must be called before using this funtion. Calling createDualStreams() enables Depth to Color sync and Image registration.
	The <B>PCLVisualizer</B> will have same parameters as that used in startPCLCloudVisualizer()*/
	void startPCLRGBCloudVisualizer();
	//!Displays the changed points in red color.
	/*!This function uses <B>pcl::octree::OctreePointCloudChangeDetector</B> class to detect the changes in point cloud. Both the streams must be enabled before calling this 
	function
	\sa startPCLCloudVisualizer()*/
	void startPointCloudChangeVisualizer();
	//!Destructor function
	/*!Destroys the streams, closes the device(s) and shutdowns the OpenNI modules*/
	~openni2_grabber();
	
protected:
	//!<B>openni::Array</B> to store the information about devices
	Array<DeviceInfo> devinfo;
	//!True if depthMode is set.
	/*!The depthMode can be set manually by calling the function setDepthMode(VideoMode mode).*/
	bool isDepthModeSet;
	//!True if colorMode is set.
	/*!The colorMode can be set manually by calling the function setColorMode(VideoMode mode).*/
	bool isColorModeSet;
	//!Ture if IRMode is set
	/*!This can be set manually by caling setIRMode(VideoMode mode)*/
	bool isIRModeSet;
	//!True if the DepthStream in started for the device
	/*!The pixel format will be of the type <B>PIXEL_FORMAT_DEPTH_1_MM</B>\see createDepthStream()*/
	bool isDepthStream;
	//!True if ColorStream is started for the device
	/*!The pixel format will be of the type <B>RGB888PIXEL</B>
	\see createColorStream()
	*/
	bool isColorStream;
	//!True if IR stream is started for the device
	/*!Pixel format will be <B>PIXEL_FORMAT_GRAY16</B>*/
	bool isIRStream;
	//!True if Depth to Color Sync and Depth to color Image Registration both are enabled
	/*!This will be set to true when createDualStreams() is called*/
	bool isSyncReg;
	//!Object of type <B>openni::Device</B> which stores device to be used
	/*!*/
	Device device;
	//!Stores the VideoStream object for the depthSensor
	/*!This object is initialized by calling createDepthStream() function*/
	VideoStream depthSensor;
	//!Stores the VideoStream object for the ColorSensor
	/*!This object is initialized by calling createColorStream() function*/
	VideoStream colorSensor;
	//!Stores the VideoStream object for the IRSensor
	/*!This object is initialized by calling createIRStream() function*/
	VideoStream IRSensor;
	//!Stores the VideoMode object for the DepthStream
	/*!This can be set by calling setColorMode(VideoMode mode) function*/
	VideoMode depthMode;
	//!Stores the VideoMode object for the ColorStream
	/*!This can be set by calling setColorMode(VideoMode mode) function*/
	VideoMode colorMode;
	//!Stores the VideoMode object for IRStream
	/*!This can be set by calling setIRMode(VideoMode mode) function*/
	VideoMode IRMode;
	//!DepthFrame data container
	/*!Contains the depth data read by calling the function <B>openni::Videostream.readFrame(&VideoFrameRef)</B>*/
	VideoFrameRef depthFrame;
	//!ColorFrame data container
	/*!Contains the color data read by calling the function <B>openni::Videostream.readFrame(&VideoFrameRef)</B>*/
	VideoFrameRef colorFrame;
	//!IR frame Data container
	/*!Contains the IR frame data read by calling the function <B>openni::Videostream.readFrame(&VideoFrameRef)</B>*/
	VideoFrameRef IRFrame;
	//!The default VideoMode for the depthStream
	/*!
	<B>Frame rate:</B> 30fps \n
	<B>Screen Resolution:</B> 640 x 480 \n
	<B>Pixel Format:</B> PIXEL_FORMAT_DEPTH_1_MM \n
	*/
	VideoMode defaultDepthMode;
	//!The default VideoMode for the colorStream
	/*!
	<B>Frame rate:</B> 30fps \n
	<B>Screen Resolution:</B> 640 x 480 \n
	<B>Pixel Format:</B> RGB888PIXEL \n
	*/
	VideoMode defaultColorMode;
	//!The default VideoMode for the colorStream
	/*!
	<B>Frame rate:</B> 30fps \n
	<B>Screen Resolution:</B> 640 x 480 \n
	<B>Pixel Format:</B> PIXEL_FORMAT_GRAY16 \n
	*/
	VideoMode defaultIRMode;
	//!Sets the VideoMode of depthStream to defaultDepthMode
	void setDepthMode();
	//!Sets the VideoMode of colorStream to defaultColorMode
	void setColorMode();
	//!Sets the VideoMode of IRStream to defaultIRMode
	void setIRMode();
};

#endif
