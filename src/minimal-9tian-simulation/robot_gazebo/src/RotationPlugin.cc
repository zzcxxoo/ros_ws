#include <string>
#include <vector>
#include <random>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

using namespace std;

namespace gazebo
{
    //  initialize class object
    vector<string> Car = {"pickup", "ambulance", "hatch", "bus", "suv", "truck"};
    vector<string> Pedestrian = {"Scrubs", "walking", "MaleVisitorOnPhone", "FemaleVisitor"};

    int getClassType(string str)
    {
        for (size_t i = 0; i < Car.size(); i++)
        {
            if (str.find(Car[i]) != str.npos) return i;
        }
        for (size_t i = 0; i < Pedestrian.size(); i++)
        {
            if (str.find(Pedestrian[i]) != str.npos) return Car.size() + i;
        }
        
        return -1;
    }


  class GAZEBO_VISIBLE RotationPlugin : public ModelPlugin
  {
    public: RotationPlugin(){}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        gzmsg << "Load RotationPlugin!!";
        this->sdf = _sdf;
        this->model = _model;
        this->world = this->model->GetWorld();

        this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&RotationPlugin::OnUpdate, this)));

        this->Reset();
    }

    // Documentation Inherited.
    public: void Reset()
    {
        this->lastUpdate = this->world->SimTime();

        if (this->sdf && this->sdf->HasElement("update_time"))
            this->update_time = this->sdf->Get<double>("update_time");
        else
            this->update_time = 0.2;

        gzmsg << "get update time: " << this->update_time << "\n";

        if (this->sdf && this->sdf->HasElement("max_update_angle"))
            this->max_update_angle = this->sdf->Get<double>("max_update_angle");
        else
            this->max_update_angle = 0.2;

        gzmsg << "get max_update_angle: " << this->max_update_angle << "\n";

        for (size_t i = 0; i < this->world->ModelCount(); i++)
        {
            physics::ModelPtr m_ptr = this->world->ModelByIndex(i);
            if(getClassType(m_ptr->GetName()) >= 0) this->model_idx.emplace_back(i);
        }
        gzmsg << "get model size: " << this->model_idx.size() << "\n";

    }

    private: void OnUpdate()
    {
        double dt = (this->world->SimTime() - this->lastUpdate).Double();

        if(dt > this->update_time)
        {
            // gzmsg << "seting pose!!\n";
            random_device rd;
            default_random_engine eng(rd());
            uniform_real_distribution<double> uni_distr(0, this->max_update_angle); 
            for (auto &&i : this->model_idx)
            {
                physics::ModelPtr mptr = this->world->ModelByIndex(i);

                ignition::math::Pose3d pose = mptr->WorldPose();
                ignition::math::Vector3d rpy = pose.Rot().Euler();
                double add_yaw = uni_distr(eng);
                ignition::math::Quaterniond new_q(rpy.X(), rpy.Y(), rpy.Z() + add_yaw);
                pose.Set(pose.Pos(), new_q);
                mptr->SetWorldPose(pose);
            }
            
            this->lastUpdate = this->world->SimTime();
        }   
        
    }

    private: vector<int> model_idx;
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: sdf::ElementPtr sdf;

    private: double update_time;
    private: double max_update_angle;

    private: std::vector<event::ConnectionPtr> connections;
    
    private: common::Time lastUpdate;
    
  };

GZ_REGISTER_MODEL_PLUGIN(RotationPlugin)
}