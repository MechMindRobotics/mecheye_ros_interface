#include "PointCloudTools.h"
#include <pcl/io/ply_io.h>
#include <opencv2/imgcodecs.hpp>

pcl::PointCloud<pcl::PointXYZ> PointCloudTools::getCloudFromDepth(cv::Mat& depth,
                                                                  const CameraIntri& intri)
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



pcl::PointCloud<pcl::PointXYZRGB> PointCloudTools::getColoredCloud(const cv::Mat& color, const cv::Mat& depth,
                                                                  const CameraIntri& intri)

{
    if (color.empty())
    {
        std::cout << "Color image is empty." << std::endl;
        return {};
    }


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

    pcl::PointCloud<pcl::PointXYZRGB> cloud(depth.cols, depth.rows);

    depth.forEach<float>([&](const float& val, const int* pos) {
        const int r = pos[0];
        const int c = pos[1];
        const pcl::PointXYZ p = mapPoint2dToPoint3d(r, c, val, intri);
        pcl::PointXYZRGB p_color;
        p_color.x = p.x;
        p_color.y = p.y;
        p_color.z = p.z;
        p_color.b = color.at<cv::Vec3b>(r,c)[0];
        p_color.g = color.at<cv::Vec3b>(r,c)[1];
        p_color.r = color.at<cv::Vec3b>(r,c)[2];

        if (pcl::isFinite(p_color))
        {
            cloud.at(c, r) = p_color;
        }
    });
    return cloud;
}




void PointCloudTools::savePointCloud(const std::string& filePath,
                                     const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PLYWriter().write(filePath, cloud);
}

void PointCloudTools::savePointCloud(const std::string& filePath,
                                     const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PLYWriter().write(filePath, cloud);
}

pcl::PointXYZ PointCloudTools::mapPoint2dToPoint3d(int r, int c, float depth, const CameraIntri &intri)
{
    if (intri.isZero()) return {};
    pcl::PointXYZ p;
    const double x = (c - intri.u);
    const double y = (r - intri.v);
    p.x = 0.001 * depth * x / intri.fx; // mm to m
    p.y = 0.001 * depth * y / intri.fy;
    p.z = 0.001 * depth;
    return p;
}
