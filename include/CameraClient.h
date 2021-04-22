#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include "ZmqClient.h"
#include "PointCloudTools.h"
#include "json/json.h"
class CameraClient : public ZmqClient
{
public:
	// Connect to camera before call other functions.
	bool connect(const std::string& ip) { return setAddr(ip, kImagePort, 60000); }

	// Depth image type: CV_32FC1
	cv::Mat captureDepthImg();

	// Color image type: CV_8UC3
	cv::Mat captureColorImg();  

	

	// Units of point cloud: meter
	pcl::PointCloud<pcl::PointXYZ> capturePointCloud();
	pcl::PointCloud<pcl::PointXYZRGB> captureRgbPointCloud();

	CameraIntri getCameraIntri();

	std::string getCameraId(); //get camera's id
	std::string getCameraVersion(); //get the version of the camera
	std::string getParameter(const std::string paraName, std::string& error); //exposed API for getting camera's parameters
	std::string setParameter(const std::string paraName, double value); //exposed API for setting camera's parameters
	Json::Value getCameraInfo();
	cv::Size getColorImgSize(); // get the height and width of color image
	cv::Size getDepthImgSize(); // get the height and width of depth image


private:
	//mmind::Response sendRequest(int command, double value);
	std::string sendRequest(std::string command, int image_type);
	Json::Value getImgSize();
	Json::Value getCameraParameter(const std::string& propertyName);
	std::string setCameraParameter(const std::string& propertyName, double value);
	const uint16_t kImagePort = 5577;
	const int SIZE_OF_JSON = 4;
};


#endif // CAMERACLIENT_H
