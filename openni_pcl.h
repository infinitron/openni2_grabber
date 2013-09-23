/*! \file openni_pcl.h*/
#ifndef OPENNI_PCL_H
#define OPENNI_PCL_H

#undef MAX
#undef MIN
#undef max
#define NOMINMAX
#include<iostream>
#include<cstdlib>
#include<cstdio>
#include <sstream>
#include <OpenNI.h>
#include <pcl/common/common_headers.h>
#include<pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<Eigen/Core>
#include <pcl/compression/octree_pointcloud_compression.h>

using namespace openni;
//!Function to act like a system pause which waits for the user to hit return key
char ReadLastCharOfLine();
//!Error handling
/*!Displays an error and returns false if status is not equal to <B>STATUS_OK</B>*/
bool handlestatus(Status status);
//!Point cloud converter
/*!Returns a point cloud of the type <B>pcl::PointCloud<pcl::PointXYZ></B>.
\param frame Object of type <B>VideoFrameRef</B> which holds a the depthPixel data
\param &stream Object of type VideoStream which holds the depthStream
\sa openni2_grabber::startPCLCloudVisualizer()
\sa openni2_grabber::getPointCloud()
*/
pcl::PointCloud<pcl::PointXYZ> cloud_xyz(VideoFrameRef frame,VideoStream &stream);
//!Prints the OpenNI version
void printver();
//!Compress the cloud data
/*!This function compresses the PointCloud data using the <B>pcl::PointCloudCompression<pcl::PointXYZ></B> class. Returns the compressed data in he form of
<B>std::stringstream</B>. The compression profile is <B>pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR</B>
\param frame Object of type VideoFrameRef which holds the depthFrame data
\param &stream Object of type VideoStream which hold the depthStream
\param showstats boolean whether to show the statistics about compression of the PointCloud. Default value is true*/
std::stringstream cloud_xyz_comp(VideoFrameRef frame,VideoStream &stream,bool showstats = true);
#endif