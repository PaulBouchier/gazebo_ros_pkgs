#ifndef _JOINT_SPEED_CTRLR_PLUGIN_H_
#define _JOINT_SPEED_CTRLR_PLUGIN_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class JointSpeedCtrlrPlugin : public ModelPlugin
  {
  public: 
    /// \brief Constructor
    JointSpeedCtrlrPlugin() {}
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(JointSpeedCtrlrPlugin)

}

#endif
