#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <string>

using namespace std;

ros::Publisher imu_pub;
ros::Publisher odom_pub;
vector<double> imu_rpy_cov, imu_drpy_cov, imu_ddxyz_cov, wheel_xyz_cov, wheel_dxyz_cov;

void imuHandler(const sensor_msgs::ImuConstPtr& msg)
{
    sensor_msgs::Imu imu = *msg;
    imu.header.frame_id = "imu_link";
    for (size_t i = 0; i < 3; i++)
    {
        imu.orientation_covariance[4 * i] = imu_rpy_cov[i];
        imu.angular_velocity_covariance[4 * i] = imu_drpy_cov[i];
        imu.linear_acceleration_covariance[4 * i] = imu_ddxyz_cov[i];
    }
    imu_pub.publish(imu);
}

void odomHandler(const nav_msgs::OdometryConstPtr& msg)
{
    nav_msgs::Odometry odom = *msg;
    odom.header.frame_id = "odom_raw";
    odom.child_frame_id = "can_ipc";
     for (size_t i = 0; i < 3; i++)
    {
        odom.pose.covariance[7 * i] = wheel_xyz_cov[i];
        odom.pose.covariance[21 + 7 * i] = wheel_dxyz_cov[i];
    }
    odom_pub.publish(odom);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "remap_topic_node");
    ros::NodeHandle nh;
    nh.param<vector<double>>("imu/rpy_cov", imu_rpy_cov, vector<double>());
    nh.param<vector<double>>("imu/drpy_cov", imu_drpy_cov, vector<double>());
    nh.param<vector<double>>("imu/ddxyz_cov", imu_ddxyz_cov, vector<double>());
    nh.param<vector<double>>("wheel/xyz_cov", wheel_xyz_cov, vector<double>());
    nh.param<vector<double>>("wheel/dxyz_cov", wheel_dxyz_cov, vector<double>());

    ros::Subscriber imu_sub = nh.subscribe("/sim/imu/data", 1, imuHandler);
    ros::Subscriber odom_sub = nh.subscribe("/nav/odom", 1, odomHandler);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom/raw", 1);

    ros::spin();
    return 0;
}
