
#include <selfie_obstacle_detection/RemoveBackground.h>

int main(int argc,char* argv[]) {
    ros::init(argc, argv, "remove_background");
    selfie_obstacle_detection::RemoveBackgroundNodelet rn;
    ros::spin();
}