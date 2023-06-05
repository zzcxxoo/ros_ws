#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

void cb(const topic_tools::ShapeShifter::ConstPtr& msg){
    ROS_WARN("recv topic!!");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_sub_any_type");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/any_type", 1, cb);

    ros::spin();
    return 0;
}


