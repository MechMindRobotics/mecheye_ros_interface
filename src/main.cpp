#include <MechMindCamera.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mechmind_camera");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    mecheye_ros_interface::MechMindCamera mm_camera;

    ros::waitForShutdown();
    return 0;
}
