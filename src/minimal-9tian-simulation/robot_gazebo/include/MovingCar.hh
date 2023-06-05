#ifndef GAZEBO_PLUGINS_MOVINGCAR_HH_
#define GAZEBO_PLUGINS_MOVINGCAR_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE MovingCar : public ModelPlugin
  {
    /// \brief Constructor
    public: MovingCar();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    private: void OnUpdate();

    private: physics::ModelPtr model;

    private: physics::WorldPtr world;

    private: sdf::ElementPtr sdf;

    private: ignition::math::Vector3d velocity;

    private: std::vector<event::ConnectionPtr> connections;
    
    private: ignition::math::Vector3d startPoint;
    private: ignition::math::Vector3d endPoint;

    private: bool forth_flag;

    private: common::Time lastUpdate;
    private: double animationFactor = 1.0;
    
  };
}
#endif