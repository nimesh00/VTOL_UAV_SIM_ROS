#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Timer.hh>
#include <ignition/math.hh>
#include <vector>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include "rosgraph_msgs/Clock.h"

#define MAX_SERVO_TRQ 10
#define T2 1.2061e-7
#define T1 -4.1556e-7
#define T0 -1.9043e-6

#define Q2 1.3114e-9
#define Q1 -2.8595e-10
#define Q0 -2.3435e-8

#define D 0.254 // 0.0254 * 10"
#define pitch 0.127 // 0.0254 * 4.5"
#define rho 1.225 // air density in Kg/m^3
#define l 0.2145 // 0.629 / 2 - 0.10
// #define As(r) D^2 * (1 + a(r)) / (1 + 2 * a(r))
#define Verr 0.1
/* For Chosen Verr */
#define A1 0.00286
#define A0 2.9948
/* *************** */
#define As 2.06e-2 // D * 0.0811021
#define Ae 2.4e-2 // 0.0811021 * 0.199357 m^2
#define Awx 3.8e-2 // 0.20 * 0.1914711 m^2
#define Awy 8.177e-3 // 0.03 * 0.2725732 m^2
#define Awz 8.0e-3 // 2 * 0.02 * 0.20

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class TernAeroPlugin : public ModelPlugin
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
        physics::Link_V links;
        physics::Joint_V joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    float curr_time, prev_time;

    ros::Subscriber actuators_sub[4];

    ros::Subscriber timer;

    rosgraph_msgs::Clock time_var;

    int seq;

    /// \brief Constructor
    public: TernAeroPlugin() {}    

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
        this -> links = _model -> GetLinks();
        this -> joint = _model -> GetJoints();

        for (int i = 0; i < this -> links.size(); i++) {
            std::cout << "Link: " << this -> links[i] -> GetName() << " at index: " << i << std::endl;
        }

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
        #else
        this->node->Init(this->model->GetWorld()->Name());
        #endif

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

        solr = ros::SubscribeOptions::create<rosgraph_msgs::Clock>(
            "/clock", 
            1,
            boost::bind(&TernAeroPlugin::time_cb, this, _1),
            ros::VoidPtr(), &this -> rosQueue);
        this -> timer = this -> rosNode -> subscribe(solr);

        // Spin up the queue helper thread.
        this->rosQueueThread =
        std::thread(std::bind(&TernAeroPlugin::QueueThread, this));

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TernAeroPlugin::updateJointStates, this));

        this -> seq = 0;

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

    void time_cb(const rosgraph_msgs::Clock::ConstPtr& msg) {
        time_var = *msg;
    }

    float omega, RPM;
    float thrust, torque;
    float Vcr;
    ignition::math::Vector3d vel;

    void setForceOnRotor(int id) {
        this -> omega = this -> joint[id] -> GetVelocity(0);
        this -> RPM = abs(this -> omega * 60 / (2.0 * M_PI));
        // std::cout << "RPM: " << this -> RPM << std::endl;
        if (abs(this -> RPM) <= 1000) {
            return;
        }
        this -> vel = this -> links[0] -> RelativeLinearVel();
        this -> Vcr = pitch * RPM / 60;
        // std::cout << "RPM: " << this -> RPM << std::endl;
        this -> thrust = (T2 * RPM * RPM + T1 * RPM + T0) * (1 - (vel.Z()) / Vcr);
        this -> torque = (this -> omega < 0 ? -1 : 1) * (Q2 * RPM * RPM + Q1 * RPM + Q0) * (1 - (vel.Z()) / Vcr);
        // std::cout << "Applied Thrust: " << this -> thrust << std::endl;
        if (!isnan(this -> torque)) {
            std::cout << "Applied Torque: " << this -> torque << std::endl;
        }
        this -> links[2 + id] -> AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
        // std::cout << "Force on body: " << this -> links[0] -> RelativeForce() << std::endl;
        this -> links[2 + id] -> AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
        // std::cout << "Torque on body: " << this -> links[0] -> RelativeTorque() << std::endl;
    }


    double FzR, FxR;
    double FzM, FxM;
    double Fx, Fz;
    double phi;
    double delta;
    double Vnet;
    double Vs;
    double a;

    void setForceOnElevon(int id, int r_id, int l_id) {
        this -> Fx = this -> FxM = this -> FxR = 0;
        this -> Fz = this -> FzM = this -> FzR = 0;
        this -> torque = 0;
        this -> omega = this -> RPM = this -> delta = this -> Vnet = this -> Vs = this -> a = 0;

        this -> omega = this -> joint[r_id] -> GetVelocity(0);
        this -> delta = -1 * this -> joint[id] -> Position();
        this -> RPM = this -> omega * 60 / (2.0 * M_PI);
        if (!isnan(this -> RPM)) {
            // std::cout << "RPM: " << this -> RPM << std::endl;
        }
        // std::cout << "Delta: " << this -> delta << std::endl;
        this -> vel = this -> links[0] -> RelativeLinearVel();
        this -> Vnet = this -> vel.Length();
        this -> a = A1 * RPM / (Vnet + Verr) + A0;
        this -> Vs = (this -> vel.Z() + Verr) * (1 + a);
        if (this -> RPM != 0) {
            this -> FzR = -1 * rho * As * (this -> Vs * this -> Vs) * sin(this -> delta) * sin(this -> delta);
            this -> FxR = -1 * rho * As * (this -> Vs * this -> Vs) * sin(this -> delta) * cos(this -> delta);
        }
        this -> phi = acos(this -> vel.X() / this -> Vnet) - this -> delta;
        this -> FzM = -1 * rho * Ae * (this -> Vnet * this -> Vnet) * cos(this -> phi) * cos(this -> phi) * sin(this -> delta);
        this -> FxM = -1 * rho * Ae * (this -> Vnet * this -> Vnet) * cos(this -> phi) * cos(this -> phi) * cos(this -> delta);
        this -> Fz = this -> FzR + this -> FzM;
        this -> Fx = this -> FxR + this -> FxM;
        this -> torque = -1 * 0.045 * (Fx * cos(this -> delta) + Fz * sin(this -> delta));
        if (isnan(this -> Fx) || isnan(this -> Fz) || isnan(this -> torque)) {
            return;
        }
        // std::cout << "Slipstream Velocity: " << this -> Vs << std::endl;
        // std::cout << "Z Velocity: " << this -> vel.Z() << std::endl;
        // std::cout << "Z force: " << this -> Fz << std::endl;
        // std::cout << "X force: " << this -> Fx << std::endl;
        // std::cout << "Torque: " << this -> torque << std::endl;
        // this -> links[l_id] -> AddLinkForce(ignition::math::Vector3d(Fx, 0, Fz), ignition::math::Vector3d(0, 0, -0.045));
        this -> links[l_id] -> AddRelativeForce(ignition::math::Vector3d(this -> Fx, 0, this -> Fz));
        this -> links[l_id] -> AddRelativeTorque(ignition::math::Vector3d(0, torque, 0));
        // std::cout << "Force on Elevon " << id << ": " << this -> links[l_id] -> RelativeForce() << std::endl;
    }

    float FbT[3];
    float TbR[3];
    float thetab[3];
    ignition::math::Vector3d body_omega;
    void setBodyForces() {
        this -> vel = this -> links[0] -> RelativeLinearVel();
        this -> FbT[0] = -1 * rho * 2 * Awx * abs(this -> vel.X()) * this -> vel.X();
        this -> FbT[1] = -1 * rho * Awy * abs(this -> vel.Y()) * this -> vel.Y();
        this -> FbT[2] = -1 * rho * 2 * Awz * abs(this -> vel.Z()) * this -> vel.Z();        
        this -> links[0] -> AddRelativeForce(ignition::math::Vector3d(FbT[0], FbT[1], FbT[2]));

        this -> body_omega = this -> links[0] -> RelativeAngularVel();
        // std::cout << "Body Angular Vel: " << this -> body_omega << std::endl;
        this -> TbR[3] = -2 * rho * Awx * l * l * l * abs(this -> body_omega.Z()) * this -> body_omega.Z();
        this -> links[0] -> AddRelativeTorque(ignition::math::Vector3d(0, 0, TbR[3]));
    }

    public: void updateJointStates() {
        seq++;
        if (seq < 2000) {
            return;
        }
        // std::cout << "Seq: " << seq++ << std::endl;
        this -> curr_time = time_var.clock.sec + (time_var.clock.nsec / 1000000000.0);
        setForceOnRotor(0);
        setForceOnRotor(1);
        setForceOnElevon(2, 0, 4);
        setForceOnElevon(3, 1, 5);
        setBodyForces();
        // setMotionForces();

        this -> prev_time = this -> curr_time;
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(TernAeroPlugin)
}
#endif
