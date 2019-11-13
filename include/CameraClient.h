#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include "ZmqClient.h"
#include "PointCloudTools.h"
#include "image.pb.h"
#include "cameraStatus.pb.h"

class CameraClient : public ZmqClient
{
public:
	// Connect to camera before call other functions.
    bool setIp(const std::string& ip) { return setAddr(ip, kImagePort, 60000); }

    // Depth image type: CV_32FC1
    cv::Mat captureDepthImg();

    // Color image type: CV_8UC3
    cv::Mat captureColorImg();
	
	// Units of point cloud: meter
	pcl::PointCloud<pcl::PointXYZ> capturePointCloud(const CameraIntri& intri);

	CameraIntri getCameraIntri();

	std::string getCameraId();
	std::string getCameraIp();
	std::string getCameraVersion();


private:
    mmind::Response sendRequest(int command, double value);
	mmind::CameraStatus getCameraStatus();

    const uint16_t kImagePort = 5577;
};


#endif // CAMERACLIENT_H
