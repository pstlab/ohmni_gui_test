#include <ros/ros.h>
#include <crow.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_gui");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Robot GUI..");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    std::string host;
    ros::param::get("~gui_host", host);
    int port;
    ros::param::get("~gui_port", port);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    gui.stop();
}