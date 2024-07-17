#include <csignal>
#include <MechMindCamera.h>
#include <area_scan_3d_camera/api_util.h>

void signalHandler(int signum)
{
    ROS_INFO("Interrupt signal (%d) received. Shutting down.", signum);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechmind_camera_publisher_service");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        MechMindCamera mm_camera;
        ros::spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}