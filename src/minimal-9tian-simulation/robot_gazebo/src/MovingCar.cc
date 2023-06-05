#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "MovingCar.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MovingCar)

// some global variables

/////////////////////////////////////////////////
MovingCar::MovingCar()
{
}

/////////////////////////////////////////////////
void MovingCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->sdf = _sdf;
    this->model = _model;
    this->world = this->model->GetWorld();

     this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
            std::bind(&MovingCar::OnUpdate, this)));

    gzmsg << "Loadding car!!\n";
    this->Reset();

}

void MovingCar::Reset()
{

    this->lastUpdate = 0;
    this->forth_flag = true;

    if (this->sdf && this->sdf->HasElement("start_point"))
        this->startPoint = this->sdf->Get<ignition::math::Vector3d>("start_point");
    else
        this->startPoint = ignition::math::Vector3d(0, 0, 0);

    if (this->sdf && this->sdf->HasElement("end_point"))
        this->endPoint = this->sdf->Get<ignition::math::Vector3d>("end_point");
    else
        this->endPoint = ignition::math::Vector3d(10, 0, 0);

    if (this->sdf && this->sdf->HasElement("velocity"))
        this->velocity = this->sdf->Get<double>("velocity");
    else
        this->velocity = 1.0;

    ignition::math::Pose3d pose = this->model->WorldPose();
    pose.Pos() = this->startPoint;
    pose.Rot() = ignition::math::Quaterniond(0, 0, 0);
    this->model->SetWorldPose(pose, false, true);

}

/////////////////////////////////////////////////
void MovingCar::OnUpdate()
{
    // Time delta
    // static double dbg_cnt = 0;
    // double dt = (_info.simTime - this->lastUpdate).Double();
    double dt = (this->world->SimTime() - this->lastUpdate).Double();
    // dbg_cnt += dt;

    ignition::math::Pose3d pose = this->model->WorldPose();
    ignition::math::Vector3d pos, rpy;
    rpy = pose.Rot().Euler();

    if(this->forth_flag){
        pos = this->endPoint - pose.Pos();
    }else
    {
        pos = this->startPoint - pose.Pos();
    }
            
    double distance = pos.Length();

    // Choose a new target position if the actor has reached its current target.
    if (distance < 0.1)
        this->forth_flag = !this->forth_flag;
    
    // Normalize the direction vector, and apply the target weight
    pos = pos.Normalize();

    // Compute the yaw orientation
    ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) - rpy.Z();
    yaw.Normalize();

    // Rotate in place, instead of jumping.
    if (yaw.Radian() > IGN_DTOR(10))
    {
        pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+
            std::max(yaw.Radian()*0.001, IGN_DTOR(0.1)));
    }
    else if(yaw.Radian() < IGN_DTOR(-10)){
        pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+
            std::min(yaw.Radian()*0.001, IGN_DTOR(-0.1)));
    }
    else
    {
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+yaw.Radian());
    }

    this->model->SetWorldPose(pose, false, true);
    
    // if(dbg_cnt > 1.0){
    //     char msg[64];
    //     sprintf(msg, "(%.3f, %.3f)", pose.Pos().X(), pose.Pos().Y());
    //     gzmsg << "setting pos: " << std::string(msg) << "\n";
    //     sprintf(msg, "(%.3f, %.3f)", this->model->WorldPose().Pos().X(), this->model->WorldPose().Pos().Y());
    //     gzmsg << "now pos: " << std::string(msg) << "\n";
    //     dbg_cnt = 0;
    // }
    this->lastUpdate = this->world->SimTime();
}