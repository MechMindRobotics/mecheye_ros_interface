#include "PointCloudTools.h"
#include <pcl/io/ply_io.h>
#include <opencv2/imgcodecs.hpp>

pcl::PointCloud<pcl::PointXYZ> PointCloudTools::getCloudFromDepth(cv::Mat& depth, const CameraIntri& intri)
{
    if (depth.empty())
    {
        std::cout << "Depth image is empty." << std::endl;
        return {};
    }
    if (depth.depth() != CV_32F)
    {
        std::cout << "Depth image data type is wrong : " << depth.depth() << std::endl;
        return {};
    }
    pcl::PointCloud<pcl::PointXYZ> cloud(depth.cols, depth.rows);
    depth.forEach<float>([&](const float& val, const int* pos) {
        const int r = pos[0];
        const int c = pos[1];
        const pcl::PointXYZ p = mapPoint2dToPoint3d(r, c, val, intri);
        if (pcl::isFinite(p))
        {
            cloud.at(c, r) = p;
        }
    });
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudTools::getRgbCloudFromDepth(cv::Mat& depth, const cv::Mat& color,
                                                                        const CameraIntri& intri)
{
    if (depth.empty() || color.empty())
    {
        std::cout << "Input image is empty." << std::endl;
        return {};
    }
    if (depth.depth() != CV_32F)
    {
        std::cout << "Depth image data type is wrong : " << depth.depth() << std::endl;
        return {};
    }
    pcl::PointCloud<pcl::PointXYZRGB> rgbCloud(depth.cols, depth.rows);
    depth.forEach<float>([&](const float& val, const int* pos) {
        const int r = pos[0];
        const int c = pos[1];
        const pcl::PointXYZ p = mapPoint2dToPoint3d(r, c, val, intri);
        if (pcl::isFinite(p))
        {
            pcl::PointXYZRGB pColor;
            pColor.x = p.x;
            pColor.y = p.y;
            pColor.z = p.z;
            pColor.r = color.at<cv::Vec3b>(r, c)[2];
            pColor.g = color.at<cv::Vec3b>(r, c)[1];
            pColor.b = color.at<cv::Vec3b>(r, c)[0];
            rgbCloud.at(c, r) = pColor;
        }
    });
    return rgbCloud;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudTools::getCloudFromDepthC3(cv::Mat& depth)
{
    if (depth.empty())
    {
        std::cout << "Depth image is empty." << std::endl;
        return {};
    }
    if (depth.depth() != CV_32F)
    {
        std::cout << "Depth image data type is wrong : " << depth.depth() << std::endl;
        return {};
    }
    pcl::PointCloud<pcl::PointXYZ> pointCloud(depth.cols, depth.rows);
    for (int r = 0; r < depth.rows; r++)
    {
        for (int c = 0; c < depth.cols; c++)
        {
            const float positionX = depth.at<cv::Vec3f>(r, c)[0];
            const float positionY = depth.at<cv::Vec3f>(r, c)[1];
            const float positionZ = depth.at<cv::Vec3f>(r, c)[2];
            if (positionZ > 1)
            {
                pcl::PointXYZ p;
                p.x = 0.001f * positionX;
                p.y = 0.001f * positionY;
                p.z = 0.001f * positionZ;
                pointCloud.at(c, r) = p;
            }
        }
    }
    return pointCloud;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudTools::getRgbCloudFromDepthC3(cv::Mat& depth, const cv::Mat& color)
{
    if (depth.empty() || color.empty())
    {
        std::cout << "Input image is empty." << std::endl;
        return {};
    }
    if (depth.depth() != CV_32F)
    {
        std::cout << "Depth image data type is wrong : " << depth.depth() << std::endl;
        return {};
    }
    pcl::PointCloud<pcl::PointXYZRGB> rgbCloud(depth.cols, depth.rows);
    for (int r = 0; r < depth.rows; r++)
    {
        for (int c = 0; c < depth.cols; c++)
        {
            const float positionX = depth.at<cv::Vec3f>(r, c)[0];
            const float positionY = depth.at<cv::Vec3f>(r, c)[1];
            const float positionZ = depth.at<cv::Vec3f>(r, c)[2];
            if (positionZ > 1)
            {
                pcl::PointXYZRGB p;
                p.x = 0.001f * positionX;
                p.y = 0.001f * positionY;
                p.z = 0.001f * positionZ;
                p.r = color.at<cv::Vec3b>(r, c)[2];
                p.g = color.at<cv::Vec3b>(r, c)[1];
                p.b = color.at<cv::Vec3b>(r, c)[0];
                rgbCloud.at(c, r) = p;
            }
        }
    }
    return rgbCloud;
}

void PointCloudTools::savePointCloud(const std::string& filePath, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PLYWriter().write(filePath, cloud);
}

void PointCloudTools::saveRgbPointCloud(const std::string& filePath, const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PLYWriter().write(filePath, cloud);
}

pcl::PointXYZ PointCloudTools::mapPoint2dToPoint3d(int r, int c, float depth, const CameraIntri& intri)
{
    if (intri.isZero())
        return {};
    pcl::PointXYZ p;
    const double x = (c - intri.u);
    const double y = (r - intri.v);
    p.x = 0.001 * depth * x / intri.fx;  // mm to m
    p.y = 0.001 * depth * y / intri.fy;
    p.z = 0.001 * depth;
    return p;
}
