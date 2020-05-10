#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

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

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection; 

    private:
        std_msgs::Float32 actuator_input[4];
        physics::Joint_V actuator_joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    ros::Subscriber leftRotorSub;
    ros::Subscriber rightRotorSub;
    ros::Subscriber leftElevonSub;
    ros::Subscriber rightElevonSub;

    ros::Subscriber actuators_sub[4];

    /// \brief Constructor
    public: TernPlugin() {}    

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
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
        this -> actuator_joint = _model -> GetJoints();


        // for (int i = 0; i < 4; i++) {
        //     this -> curr_error[i] = this -> actuator_joint[i] -> GetAngle(0).Radians();
        //     this -> prev_error[i] = this -> curr_error[i];
        // }

        // setting 5 N-m torque along the 0-axis(z-axis)
        this->actuator_joint[1]->SetForce(0, 5);


        // Set the joint's target velocity. This target velocity is just
        // for demonstration purposes.
        // this->model->GetJointController()->SetVelocityTarget(
        //     this->joint->GetScopedName(), velocity);
        // this->SetVelocity(velocity);

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

        // Create a topic name
        // std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        // this->sub = this->node->Subscribe(topicName,
        // &TernPlugin::OnActInput, this);

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
        ros::SubscribeOptions solr;

        for (int i = 0; i < 4; i++) {
            solr = ros::SubscribeOptions::create<std_msgs::Float32>(
                "/" + this -> model -> GetName() + "/" + this -> actuator_joint[i] -> GetName(), 
                1,
                boost::bind(&TernPlugin::get_act_data, this, _1, i),
                ros::VoidPtr(), &this -> rosQueue);
            this -> actuators_sub[i] = this -> rosNode -> subscribe(solr);
        }

        // solr = ros::SubscribeOptions::create<std_msgs::Float32>(
        //     "/" + this -> model -> GetName() + "/" + this -> actuator_joint[0] -> GetName(), 
        //     1,
        //     boost::bind(&TernPlugin::left_rotor_sub, this, _1),
        //     ros::VoidPtr(), &this -> rosQueue);
        // this -> leftRotorSub = this -> rosNode -> subscribe(solr);

        // solr = ros::SubscribeOptions::create<std_msgs::Float32>(
        //     "/" + this -> model -> GetName() + "/" + this -> actuator_joint[1] -> GetName(), 
        //     1,
        //     boost::bind(&TernPlugin::right_rotor_sub, this, _1),
        //     ros::VoidPtr(), &this -> rosQueue);
        // this -> rightRotorSub = this -> rosNode -> subscribe(solr);

        // solr = ros::SubscribeOptions::create<std_msgs::Float32>(
        //     "/" + this -> model -> GetName() + "/" + this -> actuator_joint[2] -> GetName(), 
        //     1,
        //     boost::bind(&TernPlugin::left_elevon_sub, this, _1),
        //     ros::VoidPtr(), &this -> rosQueue);
        // this -> leftElevonSub = this -> rosNode -> subscribe(solr);

        // solr = ros::SubscribeOptions::create<std_msgs::Float32>(
        //     "/" + this -> model -> GetName() + "/" + this -> actuator_joint[3] -> GetName(), 
        //     1,
        //     boost::bind(&TernPlugin::right_elevon_sub, this, _1),
        //     ros::VoidPtr(), &this -> rosQueue);
        // this -> rightElevonSub = this -> rosNode -> subscribe(solr);


        // ros::SubscribeOptions so =
        // ros::SubscribeOptions::create<std_msgs::Float32Ptr>(
        //     "/" + this->model->GetName() + "/vel_cmd",
        //     1,
        //     std::bind(&TernPlugin::OnActInput, this, _1),
        //     ros::VoidPtr(), &this->rosQueue);
        // this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread =
        std::thread(std::bind(&TernPlugin::QueueThread, this));

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TernPlugin::updateJointStates, this));

        // prev_time = common::time::float();
        // curr_time = prev_time;
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

    void get_act_data(const std_msgs::Float32::ConstPtr& msg, int i) {
        this -> actuator_input[i] = *msg;
    }

    // void left_rotor_sub(const std_msgs::Float32::ConstPtr& msg) {
    //     this -> actuator_input[0] = *msg;
    //     std::cout << "value changed to : " << actuator_input[0].data << "\n";
    // }

    // void right_rotor_sub(const std_msgs::Float32::ConstPtr& msg) {
    //     this -> actuator_input[1] = *msg;
    // }

    // void left_elevon_sub(const std_msgs::Float32::ConstPtr& msg) {
    //     this -> actuator_input[2] = *msg;
    // }

    // void right_elevon_sub(const std_msgs::Float32::ConstPtr& msg) {
    //     this -> actuator_input[3] = *msg;
    // }

    // private: void OnActInput(const std_msgs::Float32Ptr msg) {
    //     this -> actuator_input = msg;
    // }

    void elevonControlRoutine(int id) {
        // float curr_error = this -> actuator_joint[id] -> GetAngle(0).Radians();
        // float control_torque = Kp_e * error + Kd_e * 
    }

    public: void updateJointStates() {
        // this -> actuator_input = data;
        // Joint 0 and 1 are motor joints (Left and Right)
        // Joint 2 and 3 are elevon joints (Left and Right)
        
        std::cout << "Actuator Inputs:\n";

        for (int i = 0; i < 4; i++) {
            std::cout << actuator_input[i].data << " ";
        }

        std::cout << "\n";
        // elevonControlRoutine(2);
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TernPlugin)
}
#endif
