#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class TernPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TernPlugin() {}
    float control_input[4];

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        std::cout << "Load function ended!!\n";
        // Safety check
        if (_model->GetJointCount() == 0)
        {
        std::cerr << "Invalid joint count, tern plugin not loaded\n";
        return;
        }

        // Store the model pointer for convenience.
        this->model = _model;

        // Get the first joint. We are making an assumption about the model
        // having one joint that is the rotational joint.
        this->joint[4] = _model->GetJoints();

        this->joint2 = model->GetJoints()[1];

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(0.1, 0, 0);

        // Apply the P-controller to the joint.
        this->model->GetJointController()->SetVelocityPID(
            this->joint->GetScopedName(), this->pid);


        // setting 5 N-m torque along the 0-axis(z-axis)
        this->joint2->SetForce(0, 5);

        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        // Default to zero velocity
        double velocity = 0;

        // Check that the velocity element exists, then read the value
        if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");


        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        // this->model->GetJointController()->SetVelocityTarget(
        //     this->joint->GetScopedName(), velocity);
        this->SetVelocity(velocity);

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        this->sub = this->node->Subscribe(topicName,
        &TernPlugin::OnMsg, this);

        if (!ros::isInitialized())
        {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&TernPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread =
        std::thread(std::bind(&TernPlugin::QueueThread, this));

        std::cout << "Load function ended!!\n";
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), _vel);
    }

    private: void SetJointVelocity(physics::JointPtr Joint, const double vel) {
        
    }


    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
        this->SetVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
    this->SetVelocity(_msg->x());
    }

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;


    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    // private: physics::JointPtr joint;

    // private: physics::JointPtr joint2;

    private: physics::JointPtr joint[4];

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TernPlugin)
}
#endif
