#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Timer.hh>
#include <vector>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include "rosgraph_msgs/Clock.h"

#define MAX_SERVO_TRQ 10

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
        float actuator_input[4];
        physics::Joint_V actuator_joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    float curr_time, prev_time;

    // vector<vector<float>> PID_c;
    float Kp[4], Kd[4], Ki[4];

    ros::Subscriber actuators_sub[4];

    ros::Subscriber timer;

    rosgraph_msgs::Clock time_var;

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

        // PID constants for actuators (Kp, Kd, Ki)
        // PID_c = {{10.0, 1.0, 0.0},
        //          {10.0, 1.0, 0.0},
        //          {10.0, 1.0, 0.0},
        //          {10.0, 1.0, 0.0}};
        for (int j = 0; j < 4; j++) {
            Kp[j] = 30.0;
            Kd[j] = 1.0;
            Ki[j] = 0.0;
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
        // this->actuator_joint[2]->SetForce(1, 5);


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

        solr = ros::SubscribeOptions::create<rosgraph_msgs::Clock>(
            "/clock", 
            1,
            boost::bind(&TernPlugin::time_cb, this, _1),
            ros::VoidPtr(), &this -> rosQueue);
        this -> timer = this -> rosNode -> subscribe(solr);

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
        this -> actuator_input[i] = msg -> data;
    }

    void time_cb(const rosgraph_msgs::Clock::ConstPtr& msg) {
        time_var = *msg;
    }

    
    float curr_error[4];
    float prev_error[4];
    float delta_t;
    float control_torque;
    float control_torque_tmp;

    float get_control_torque_elevon(int id) {
        this -> control_torque_tmp = this -> Kp[id] * this -> curr_error[id] + this -> Kd[id] * (float)(this -> curr_error[id] - this -> prev_error[id]) / this -> delta_t;
        if (this -> control_torque_tmp > MAX_SERVO_TRQ) {
            return MAX_SERVO_TRQ;
        } else if (this -> control_torque_tmp < -MAX_SERVO_TRQ) {
            return -MAX_SERVO_TRQ;
        } else {
            return this -> control_torque_tmp;
        }
    }

    void elevonControlRoutine(int id) {
        this -> curr_error[id] = -1 * (this -> actuator_joint[id] -> Position() - this -> actuator_input[id]);
        // std::cout << "Current Error: " << this -> curr_error[id] << "\n";
        if (this -> curr_error[id] < 0.001 && this -> curr_error[id] > -0.001) {
            return;
        }
        // std::cout << "Current Angle & Target Angle: " << this -> actuator_joint[id] -> Position() << " " << this -> actuator_input[id] << std::endl;
        // std::cout << "Current Time Step: " << time_var.clock.sec + (time_var.clock.nsec / 1000000000.0) << "\n";
        this -> delta_t = this -> curr_time - this -> prev_time;
        if (this -> delta_t == 0) {
            return;
        }
        // std::cout << "Error Rate: " << (this -> curr_error[id] - this -> prev_error[id]) / this -> delta_t << "\n";
        // std::cout << "delta_t: " << delta_t << "\n";
        // PD control
        this -> control_torque = get_control_torque_elevon(id);
        std::cout << "Contorl Torque: " << this -> control_torque << "\n";
        this -> actuator_joint[id] -> SetForce(0, this -> control_torque);
        // this->actuator_joint[id]->SetForce(1, 5);
        this -> prev_error[id] = this -> curr_error[id];
        return;
    }

    public: void updateJointStates() {
        this -> curr_time = time_var.clock.sec + (time_var.clock.nsec / 1000000000.0);
        // this -> actuator_input = data;
        // Joint 0 and 1 are motor joints (Left and Right)
        // Joint 2 and 3 are elevon joints (Left and Right)
        
        // std::cout << "Actuator Inputs:\n";

        // for (int i = 0; i < 4; i++) {
        //     std::cout << actuator_input[i].data << " ";
        // }

        // std::cout << "\n";
        elevonControlRoutine(2);
        elevonControlRoutine(3);

        this -> prev_time = this -> curr_time;
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TernPlugin)
}
#endif
