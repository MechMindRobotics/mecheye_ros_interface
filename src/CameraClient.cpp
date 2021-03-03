  #include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "CameraCmd.h"
#include <regex>
#include "json/json.h"
#define INT32 4
namespace
{
	double readDouble(const std::string& data, const int pos)
	{
		if (pos + sizeof(double) > data.size()) return 0;
		std::string strFromQDataStream(data.data() + pos, sizeof(double));
		std::string str;
		str.resize(sizeof(double));
		for (int i = 0; i < sizeof(double); i++)
		{
			str[i] = strFromQDataStream[sizeof(double) - 1 - i];
		}
		double v;
		memcpy(&v, str.c_str(), sizeof(double));
		return v;
	}

	int readInt(const std::string& data, const int pos)
	{
		if (pos + sizeof(INT32) > data.size()) return 0;
		std::string strFromQDataStream(data.data() + pos, sizeof(INT32));
		std::string str;
		str.resize(sizeof(INT32));
		for (int i = 0; i < sizeof(INT32); i++)
		{
			str[i] = strFromQDataStream[sizeof(INT32) - 1 - i];
		}
		int v;
		memcpy(&v, str.c_str(), sizeof(INT32));
		return v;
	}

	cv::Mat asMat(const std::string& data, size_t offset = 0)
	{
		return cv::Mat(data.size() - offset, 1, cv::DataType<uchar>::type,
			const_cast<char*>((data.data() + offset)));
	}
	cv::Mat matC1ToC3(const cv::Mat& matC1)
	{
		if (matC1.empty())
			return cv::Mat();
		if (matC1.channels() != 1 || (matC1.rows % 3) != 0)
			return cv::Mat();

		std::vector<cv::Mat> channels;
		for (int i = 0; i < 3; i++) {
			channels.push_back(matC1(cv::Rect(0, i * matC1.rows / 3, matC1.cols, matC1.rows / 3)));
		}
		cv::Mat matC3;
		cv::merge(channels, matC3);
		return matC3;
	}
	cv::Mat read32FC1Mat(const std::string& data, double scale)
	{
		if (data.empty()) return {};
		cv::Mat bias16U =
			cv::imdecode(asMat(data), cv::ImreadModes::IMREAD_ANYDEPTH);
		cv::Mat bias32F = cv::Mat::zeros(bias16U.size(), CV_32FC1);
		bias16U.convertTo(bias32F, CV_32FC1);
		cv::Mat mat32F =
			bias32F + cv::Mat(bias32F.size(), bias32F.type(), cv::Scalar::all(-Encode32FBias));
		return scale == 0 ? cv::Mat() : mat32F / scale;
	}
	
	cv::Mat read32FC3Mat(const std::string& data, double scale)
	{
		if (data.empty()) return cv::Mat();
		cv::Mat matC1 = cv::imdecode(asMat(data), cv::ImreadModes::IMREAD_ANYDEPTH);
		cv::Mat bias16UC3 = matC1ToC3(matC1);
		cv::Mat bias32F = cv::Mat::zeros(bias16UC3.size(), CV_32FC3);
		
		bias16UC3.convertTo(bias32F, CV_32FC3);
		cv::Mat depth32F =
			bias32F + cv::Mat(bias32F.size(), bias32F.type(), cv::Scalar::all(-Encode32FBias));
		cv::Mat rel = depth32F / scale;	
		return rel;
	}

	vector<string> splitStr(const string& str, const string& delim) {
		vector<string> res;
		if ("" == str) return res;
		char* strs = new char[str.length() + 1];
		strcpy(strs, str.c_str());

		char* d = new char[delim.length() + 1];
		strcpy(d, delim.c_str());

		char* p = strtok(strs, d);
		while (p) {
			string s = p;
			res.push_back(s);
			p = strtok(NULL, d);
		}

		return res;
	}

	vector<int> find_number_pos(const string &str) {
		vector<int> rel;
		if (str.size() == 0) {
			rel.push_back(-1);
			rel.push_back(-1);
		}
		else {
			for (int i = 0; i < str.size(); i++)
			{
				if (str[i] >= '0' && str[i] <= '9') {
					rel.push_back(i);
					break;
				}
			}
			for (int i = str.size() - 1; i >= 0; i--)
			{
				if (str[i] >= '0' && str[i] <= '9') {
					rel.push_back(i);
					break;
				}
			}
		}
		return rel;
	}
	
}

cv::Mat CameraClient::captureDepthImg()
{
	std::string response = sendRequest(Command::CaptureImage, ImageType::DEPTH);
	int jsonSize = readInt(response,0);
	double scale = readDouble(response, jsonSize + SIZE_OF_JSON);
	int imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE);
	int imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + sizeof(INT32);
	std::string imageDepth = response.substr(imageBegin, imageSize);
	if (imageDepth.size() == 0)
	{
		std::cout << "Client depth image is empty!" << std::endl;
		return {};
	}
	std::cout << "Depth image captured!" << std::endl;
	return read32FC1Mat(imageDepth, scale);
}

cv::Mat CameraClient::captureColorImg()
{
	
	std::string response = sendRequest(Command::CaptureImage, ImageType::COLOR);
	int jsonSize = readInt(response, 0);
	int imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE);
	int imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + sizeof(INT32);
	std::string imageRGB = response.substr(imageBegin, imageSize);
	if (imageRGB.size() == 0)
	{
		std::cout << "Client color image is empty!" << std::endl;
		return {};
	}
	std::cout << "Color image captured!" << std::endl;
	return cv::imdecode(asMat(imageRGB), cv::ImreadModes::IMREAD_COLOR);
}

pcl::PointCloud<pcl::PointXYZRGB> CameraClient::captureRgbPointCloud()
{
	std::string response = sendRequest(Command::CaptureGratingImage, ImageType::MatXYZ);
	int jsonSize = readInt(response, 0);
	double scale = readDouble(response, jsonSize + SIZE_OF_JSON);
	int imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE);
	int imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + sizeof(INT32);
	std::string imageDepth = response.substr(imageBegin, imageSize);
	cv::Mat depthC3 = read32FC3Mat(imageDepth,scale);
	const cv::Mat color = captureColorImg();
	return PointCloudTools::getRgbCloudFromDepthC3(depthC3, color);
}

pcl::PointCloud<pcl::PointXYZ> CameraClient::capturePointCloud()
{
	std::string response = sendRequest(Command::CaptureGratingImage, ImageType::MatXYZ);
	int jsonSize = readInt(response, 0);
	double scale = readDouble(response, jsonSize + SIZE_OF_JSON);
	int imageSize = readInt(response, SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE);
	int imageBegin = SIZE_OF_JSON + jsonSize + SIZE_OF_SCALE + sizeof(INT32);
	std::string imageDepth = response.substr(imageBegin, imageSize);
	cv::Mat depthC3 = read32FC3Mat(imageDepth, scale);
	return PointCloudTools::getCloudFromDepthC3(depthC3);
}

CameraIntri CameraClient::getCameraIntri()
{
	std::string response = sendRequest(Command::GetCameraIntri, 0);	
	Json::Reader reader;
	Json::Value info;
	reader.parse(response.substr(SIZE_OF_JSON, response.size() - SIZE_OF_JSON), info);
	Json::Value intriValue = info["camera_intri"]["intrinsic"];
	std::string originStr = intriValue.toStyledString();
	vector<int> pos = find_number_pos(originStr);
	vector<std::string> intriArray = splitStr(originStr.substr(pos[0],pos[1] - pos[0]+1), ",");
	CameraIntri intri;
	intri.fx = std::stod(intriArray[0].c_str());
	intri.fy = std::stod(intriArray[1].c_str());
	intri.u = std::stod(intriArray[2].c_str());
	intri.v = std::stod(intriArray[3].c_str());

	std::cout.precision(17);
	std::cout << "fx = " << intri.fx << std::endl
		<< "fy = " << intri.fy << std::endl
		<< "u = " << intri.u << std::endl
		<< "v = " << intri.v << std::endl;

	return intri;
}

std::string CameraClient::getCameraId()
{
	return  getCameraInfo()["eyeId"].toStyledString();
}

std::string CameraClient::getCameraVersion()
{
	return  getCameraInfo()["version"].toStyledString();
}

std::string CameraClient::getParameter(std::string paraName, std::string& error)
{
	Json::Value configs =  getCameraParameter(paraName);
	if (configs.isMember(paraName)) {
		return configs[paraName].toStyledString();
	}
	else {
		error = "Property not exist";
		return "";
	}
}

std::string  CameraClient::setParameter(std::string paraName, double value)
{
	std::string rel = setCameraParameter(paraName, value);
	return rel;
}

Json::Value CameraClient::getCameraInfo()
{
	Json::Reader reader;
	Json::Value info;
    std::string response = sendRequest(Command::GetCameraInfo, 0);
	reader.parse(response.substr(SIZE_OF_JSON,response.size() - SIZE_OF_JSON), info);
	return info["camera_info"];
}

Json::Value CameraClient::getCameraParameter(const std::string& propertyName)
{
	Json::Value request;
	Json::FastWriter fwriter;
	Json::Value reply;
	Json::Reader reader;
	request[Service::cmd] = Command::GetCameraParams;
	request[Service::property_name] = propertyName;
	std::string response = sendReq(fwriter.write(request));;
	reader.parse(response.substr(SIZE_OF_JSON, response.size() - SIZE_OF_JSON), reply);
	Json::Value allConfigs = reply["camera_config"]["configs"][0];
	return allConfigs;
}

std::string CameraClient::setCameraParameter(const std::string& propertyName, double value)
{
	Json::Value request;
	Json::FastWriter fwriter;
	Json::Value reply;
	Json::Reader reader;
	request[Service::cmd] = Command::SetCameraParams;
	request[Service::camera_config][propertyName] = value;
	std::string response = sendReq(fwriter.write(request));
	reader.parse(response.substr(SIZE_OF_JSON, response.size() - SIZE_OF_JSON), reply);
	if (reply.isMember("err_msg"))
		return reply["err_msg"].toStyledString();
	return "";
}


std::string CameraClient::sendRequest(std::string command, int image_type)
{
	Json::Value req;
	Json::FastWriter fwriter;
	req[Service::cmd] = Json::Value(command);
	req[Service::image_type] = Json::Value(image_type);
	return sendReq(fwriter.write(req));
}
