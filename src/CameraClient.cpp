#include "CameraClient.h"
#include <opencv2/imgcodecs.hpp>
#include "CameraCmd.h"
#include <regex>

namespace {

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

cv::Mat asMat(const std::string& data, size_t offset = 0)
{
    return cv::Mat(data.size() - offset, 1, cv::DataType<uchar>::type,
                   const_cast<char*>((data.data() + offset)));
}

cv::Mat read32FC1Mat(const std::string& data, size_t offset = 0)
{
    if (data.empty()) return {};
    const double scale = readDouble(data, offset);
    cv::Mat bias16U =
        cv::imdecode(asMat(data, sizeof(double) + offset), cv::ImreadModes::IMREAD_ANYDEPTH);
    cv::Mat bias32F = cv::Mat::zeros(bias16U.size(), CV_32FC1);
    bias16U.convertTo(bias32F, CV_32FC1);
    cv::Mat mat32F =
        bias32F + cv::Mat(bias32F.size(), bias32F.type(), cv::Scalar::all(-Encode32FBias));
    return scale == 0 ? cv::Mat() : mat32F / scale;
}
}

cv::Mat CameraClient::captureDepthImg()
{
    const mmind::Response response = sendRequest(NetCamCmd::CaptureImage, ImageType::DEPTH);
    if (response.imagedepth().empty())
    {
        std::cout << "Client depth image is empty!" << std::endl;
        return {};
    }
    return read32FC1Mat(response.imagedepth(), 2);
}

cv::Mat CameraClient::captureColorImg()
{
    const mmind::Response response = sendRequest(NetCamCmd::CaptureImage, ImageType::COLOR);
    if (response.imagergb().empty())
    {
        std::cout << "Client color image is empty!" << std::endl;
        return {};
    }
    return cv::imdecode(asMat(response.imagergb()), cv::ImreadModes::IMREAD_COLOR);
}

pcl::PointCloud<pcl::PointXYZ> CameraClient::capturePointCloud(const CameraIntri& intri)
{
    cv::Mat depth = captureDepthImg();
    return PointCloudTools::getCloudFromDepth(depth, intri);
}

CameraIntri CameraClient::getCameraIntri()
{
	const mmind::Response response = sendRequest(NetCamCmd::GetCameraIntri, 0);
	std::cout << "Camera intrinsics: " << std::endl
			  << response.camintri() << std::endl;
	
	const auto start = response.camintri().find_last_of("[");
	const auto end = response.camintri().find_last_of("]");
	if (start == std::string::npos || end == std::string::npos || end < start)
	{
		std::cout << "Wrong camera intrinsics." << std::endl;
		return{};
	}
	const std::string intriStr = response.camintri().substr(start + 1, end - start - 1);

	std::regex re(",");
	std::vector<std::string> intriValue(std::sregex_token_iterator(intriStr.begin(),
		intriStr.end(), re, -1), std::sregex_token_iterator());
	if (intriValue.size() != 4)
	{
		std::cout << "Wrong intrinsics value" << std::endl;
		return{};
	}

	CameraIntri intri;
	intri.fx = std::stod(intriValue[0].c_str());
	intri.fy = std::stod(intriValue[1].c_str());
	intri.u = std::stod(intriValue[2].c_str());
	intri.v = std::stod(intriValue[3].c_str());

	std::cout.precision(17);
	std::cout << "fx = " << intri.fx << std::endl
			  << "fy = " << intri.fy << std::endl
			  << "u = " << intri.u << std::endl
			  << "v = " << intri.v << std::endl;

	return intri;
}

std::string CameraClient::getCameraId()
{
	return  getCameraStatus().eyeid();
}

std::string CameraClient::getCameraIp()
{
	return  getCameraStatus().ip();
}

std::string CameraClient::getCameraVersion()
{
	return  getCameraStatus().version();
}

mmind::CameraStatus CameraClient::getCameraStatus()
{
	const mmind::Response response = sendRequest(NetCamCmd::GetCameraStatus, 0);
	mmind::CameraStatus cameraStatus;
	if (!cameraStatus.ParseFromString(response.camerastatus()))
	{
		std::cout << "Camera status is empty!";
		return{};
	}
	return cameraStatus;
}

mmind::Response CameraClient::sendRequest(int command, double value)
{
    mmind::Request request;
    request.set_command(command);
    request.set_valuedouble(value);
    return sendReq<mmind::Response>(request);
}
