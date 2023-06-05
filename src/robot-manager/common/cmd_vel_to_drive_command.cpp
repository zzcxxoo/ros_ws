#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_platform_msgs/DriveCommand.h>

ros::Publisher cmd_pub;

std::string twist_cmd_topic = "/cmd_vel";
std::string drive_cmd_topic = "/drive_command";
float wheelbase = 0.806;
std::string frame_id = "odom";

float convert_trans_rot_vel_to_steering_angle(float v, float omega)
{
    if(omega == 0 || v == 0)
        return 0;
    
    float radius = v / omega;
    return std::atan(wheelbase / radius) * 57.29578;

}


void cmd_callback(const geometry_msgs::TwistConstPtr& data)
{
  
    float v = data->linear.x;
    float steering = convert_trans_rot_vel_to_steering_angle(v, data->angular.z);
    
    // if steering >= 35:
    //   steering = 35
    // if steering <= -35:
    //   steering = -35
    // rospy.loginfo("steering_after:%f", steering)

    mobile_platform_msgs::DriveCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    //   msg.front_wheel_angle = (-1)*steering
    msg.front_wheel_angle = steering;
    msg.speed_mps = v;
    cmd_pub.publish(msg);
  
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_vel_to_drive_command_node");
    ros::NodeHandle nh;

    ros::Subscriber twist_sub;

    twist_sub = nh.subscribe(twist_cmd_topic, 1, cmd_callback);
    cmd_pub = nh.advertise<mobile_platform_msgs::DriveCommand>(drive_cmd_topic, 1);
    
    ros::spin();

    return 0;
}
