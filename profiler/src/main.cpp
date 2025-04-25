#include <csignal>
#include <MechMindProfiler.h>

void signalHandler(int signum)
{
    ROS_INFO("Interrupt signal (%d) received. Shutting down.", signum);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechmind_profiler_publisher_service");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        MechMindProfiler mm_profiler;
        ros::spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}