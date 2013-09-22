#include "openni_pcl.h"

char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	std::cout<<"Press enter to continue..\n";
	do
	{
		lastChar = newChar;
		newChar = getchar();
	}
	while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;
}

bool handlestatus(Status status)
{
    if(status!=STATUS_OK)
    {
        std::cout<<"Error: "<<OpenNI::getExtendedError()<<"\n";
		return false;
    }
	else return true;

}


pcl::PointCloud<pcl::PointXYZ> cloud_xyz(VideoFrameRef frame,VideoStream &stream)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
    if(frame.getData() == NULL) std::cout<<"Empty data\n..";
    float px,py,pz;
    DepthPixel *depthp,dp;
    Status status;
    cloud.width = frame.getWidth();
    cloud.height = frame.getHeight();
    cloud.resize(cloud.height * cloud.width);
	int count=0;
	depthp = (DepthPixel*)((char*)frame.getData());
    for(int y=0;y<frame.getHeight();y++)
    {
        for(int x=0;x<frame.getWidth();x++)
        {
            dp=*depthp;
            status = CoordinateConverter::convertDepthToWorld(stream,x,y,dp,&px,&py,&pz);
            if(!handlestatus(status)) exit(0);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].x = (px);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].y = (py);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].z = (pz);
            depthp++;
        }
    }
	return cloud;
}

void printver()
{
    printf("OpenNI Version is %d.%d.%d.%d\n",
    OpenNI::getVersion().major,
    OpenNI::getVersion().minor,
    OpenNI::getVersion().maintenance,
    OpenNI::getVersion().build);
}

std::stringstream cloud_xyz_comp(VideoFrameRef frame,VideoStream &stream,bool showstats)
{
	std::stringstream comp_data;
	pcl::PointCloud<pcl::PointXYZ> cloud;
    if(frame.getData() == NULL) std::cout<<"Empty data\n..";
    float px,py,pz;
    DepthPixel *depthp,dp;
    Status status;
    cloud.width = frame.getWidth();
    cloud.height = frame.getHeight();
    cloud.resize(cloud.height * cloud.width);
	int count=0;
	depthp = (DepthPixel*)((char*)frame.getData());
    for(int y=0;y<frame.getHeight();y++)
    {
        for(int x=0;x<frame.getWidth();x++)
        {
            dp=*depthp;
            status = CoordinateConverter::convertDepthToWorld(stream,x,y,dp,&px,&py,&pz);
            if(!handlestatus(status)) exit(0);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].x = (px);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].y = (py);
            cloud.points[x + (frame.getStrideInBytes() * y)/2].z = (pz);
            depthp++;
        }
    }
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr const_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud));
	pcl::octree::PointCloudCompression<pcl::PointXYZ> *encoder;
	pcl::octree::compression_Profiles_e comp_profile = pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
	encoder = new pcl::octree::PointCloudCompression<pcl::PointXYZ>(comp_profile,showstats);
	encoder->encodePointCloud(const_cloud,comp_data);
	return comp_data;
}