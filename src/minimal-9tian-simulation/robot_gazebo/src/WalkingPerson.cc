/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "WalkingPerson.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(WalkingPerson)

#define WALKING_ANIMATION "walking"

// #define DIST_FROM_ORIGIN(x, y, ox, oy) ((x-ox)*(x-ox) + (y-oy)*(y-oy))
#define PERSON_Z 1.05
#define MIN_DIST_FROM_LAST_TARGET 1.0

/////////////////////////////////////////////////
WalkingPerson::WalkingPerson()
{
}

/////////////////////////////////////////////////
void WalkingPerson::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&WalkingPerson::OnUpdate, this, std::placeholders::_1)));

  gzmsg << "Loadding actor!!\n";
  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void WalkingPerson::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, PERSON_Z);

  this->center = this->target;

  if (this->sdf && this->sdf->HasElement("walking_range"))
    this->walkingRange = this->sdf->Get<double>("walking_range");
  else
    this->walkingRange = 10.0;

  gzmsg << "getting walkingRange: " << this->walkingRange << "\n";

  ignition::math::Pose3d pose = this->actor->WorldPose();
  pose.Pos() = this->target;
  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, 0);
  this->actor->SetWorldPose(pose, false, false);

  char msg[64];
  sprintf(msg, "(%.3f, %.3f)", this->target.X(), this->target.Y());
  gzmsg << "setting initial pos: " << std::string(msg) << "\n";

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void WalkingPerson::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < MIN_DIST_FROM_LAST_TARGET)
  {
    double r = this->walkingRange;
    double x = this->center.X() + ignition::math::Rand::DblUniform(-r, r);
    double y = this->center.Y() + ignition::math::Rand::DblUniform(-r, r);

    newTarget.X(x);
    newTarget.Y(y);

    // for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    // {
    //   double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
    //       - newTarget).Length();
    //   if (dist < 2.0)
    //   {
    //     newTarget = this->target;
    //     break;
    //   }
    // }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void WalkingPerson::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void WalkingPerson::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    char msg[64];
    sprintf(msg, "(%.3f, %.3f)", this->target.X(), this->target.Y());
    gzmsg << "New target: " << msg << "\n";
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
//   this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  // gzdbg << "delta yaw: " << yaw.Degree() <<"\n";
  yaw.Normalize();
  // gzdbg << "after delta yaw: " << yaw.Degree() <<"\n";

  // Rotate in place, instead of jumping.
  if (yaw.Radian() > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        std::max(yaw.Radian()*0.001, IGN_DTOR(0.1)));
  }
  else if(yaw.Radian() < IGN_DTOR(-10)){
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        std::min(yaw.Radian()*0.001, IGN_DTOR(-0.1)));
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  // pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  // pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(PERSON_Z);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}