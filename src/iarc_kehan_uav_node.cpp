#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarc_kehan_uav_node");
    ros::NodeHandle nh("~");

    std::cout << "\033[32m" << ros::this_node::getName() << " start!"
              << "\033[0m" << std::endl;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        clock_t time_begin = clock();

        // TODO

        loop_rate.sleep();
        ROS_ERROR("!!!!!!!!!!!!!!!fps time!!!!!!!!!!!!!!!!");
        ROS_ERROR("%lf\n\n", 1000.0 * (clock() - time_begin) / CLOCKS_PER_SEC);
    }

    return 0;
}
