#include "ros/ros.h"

int main(int argc, char *argv[]){
    // init
    ros::init(argc, argv, "Hello");
    // 
    ros::NodeHandle n;
    //
    ROS_INFO("Hello World");

    return 0;
}