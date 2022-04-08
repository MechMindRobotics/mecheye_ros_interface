#ifndef POINTCLOUDTOOLS_H
#define POINTCLOUDTOOLS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/mat.hpp>

struct CameraIntri
{
    CameraIntri() = default;
    CameraIntri(double fx, double fy, double u, double v) : fx(fx), fy(fy), u(u), v(v)
    {
    }
    double fx = 0.0;
    double fy = 0.0;
    double u = 0.0;
    double v = 0.0;
    bool isZero() const
    {
        return (fx == 0.0 && fy == 0.0 && u == 0.0 && v == 0.0);
    }
};

class PointCloudTools
{
public:
    static pcl::PointCloud<pcl::PointXYZ> getCloudFromDepth(cv::Mat& depth, const CameraIntri& intri);

    static pcl::PointCloud<pcl::PointXYZRGB> getRgbCloudFromDepth(cv::Mat& depth, const cv::Mat& color,
                                                                  const CameraIntri& intri);

    static pcl::PointCloud<pcl::PointXYZ> getCloudFromDepthC3(cv::Mat& depth);

    static pcl::PointCloud<pcl::PointXYZRGB> getRgbCloudFromDepthC3(cv::Mat& depth, const cv::Mat& color);

    static void savePointCloud(const std::string& filePath, const pcl::PointCloud<pcl::PointXYZ>& cloud);

    static void saveRgbPointCloud(const std::string& filePath, const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

private:
    static pcl::PointXYZ mapPoint2dToPoint3d(int r, int c, float depth, const CameraIntri& intri);
};
#endif  // POINTCLOUDTOOLS_H
