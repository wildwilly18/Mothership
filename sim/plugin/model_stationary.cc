#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    { 
      this->model->SetGravityMode(free);
      // Apply a small linear velocity to the model. May need to adjust X velocity
      this->model->SetLinearVel(ignition::math::Vector3d(0, 0, .0395));

      //Now taking position of the model in world pose and breaking it out by X Y Z position
      //auto position = this->model->WorldPose();
      //ignition::math::Vector3<double> pos = position.Pos();
      //printf("X_Pos: %2.2f Y_Pos: %2.2f Z_Pos: %2.2f \n", pos.X(), pos.Y(), pos.Z());
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
