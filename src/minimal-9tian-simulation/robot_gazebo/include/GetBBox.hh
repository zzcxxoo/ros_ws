#ifndef GAZEBO_GETBBOX_HH
#define GAZEBO_GETBBOX_HH

#include <iostream>

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <robot_gazebo/bboxPerFrame.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>


using namespace std;

namespace gazebo
{
class GetBBox : public ModelPlugin
{
    public: GetBBox();

    public: virtual ~GetBBox();

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected: virtual void UpdateChild();

    private: physics::WorldPtr world_;

    private: rendering::ScenePtr scene_;

    private: ros::NodeHandle* rosnode_;

    private: boost::mutex lock_;

    private: std::string robot_namespace_;

    private: bool ServiceCallback(robot_gazebo::bboxPerFrame::Request &req,
                                robot_gazebo::bboxPerFrame::Response &res);

    private: ros::ServiceServer srv_;
    private: std::string service_name_;

    private: ros::CallbackQueue imu_queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

};
}
#endif