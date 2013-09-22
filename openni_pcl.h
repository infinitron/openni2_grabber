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

char ReadLastCharOfLine();
bool handlestatus(Status status);
pcl::PointCloud<pcl::PointXYZ> cloud_xyz(VideoFrameRef frame,VideoStream &stream);
void printver();
std::stringstream cloud_xyz_comp(VideoFrameRef frame,VideoStream &stream,bool showstats = true);
#endif