#include <gazebo_plugins/gazebo_ros_joint_speed_ctrlr.h>

namespace gazebo
{
    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    void 
    JointSpeedCtrlrPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cerr << "\nThe joint velocity controller plugin is attach to model[" <<
        _model->GetName() << "]\n";

      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, joint velocity controller plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Check that the velocity element exists, then read the value
      double velocity = 0;
      if (_sdf->HasElement("velocity"))
      {
        velocity = _sdf->Get<double>("velocity");
      }

      // Check that the joint_name element exists, then read the value
      std::string joint_name;
      if (_sdf->HasElement("joint_name"))
      {
        joint_name = _sdf->Get<std::string>("joint_name");
        std::cerr << "Setting joint " << joint_name << std::endl;
      }

      // Check that the Kp element exists, then read the value
      double Kp = 0.1;
      if (_sdf->HasElement("Kp"))
      {
        Kp = _sdf->Get<double>("Kp");
      }

      // Get the rotational joint.
      this->joint = _model->GetJoint(joint_name);
      if (NULL == this->joint)
      {
        std::cerr << "Error: could not find joint name\n";
        return;
      }
      std::cerr << "Setting joint " << this->joint->GetScopedName() 
          << " velocity to: " << velocity << std::endl;

      // Setup a PID-controller, with a gain of 1.
      this->pid = common::PID(Kp, 1.0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      
      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);
    }
}

