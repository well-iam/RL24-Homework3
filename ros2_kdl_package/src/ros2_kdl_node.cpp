// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

const double traj_duration = 10;
const double dt = 0.01;
const double acc_duration = traj_duration/3;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position". That's where we choose type of control
            declare_parameter<int>("traj_sel", 0); // defaults to 0
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("traj_sel", traj_sel_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            RCLCPP_INFO(get_logger(),"Current trajectory is: '%d'", traj_sel_);

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort_joint" ||cmd_interface_ == "effort_operational"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }
            if (traj_sel_ < 0 || traj_sel_ > 3)
            {
                RCLCPP_INFO(get_logger(),"Selected trajectory is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"}); //In parameter we have the robot description

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            std::cout << "Nr of joints: " << nj << std::endl;

            KDL::JntArray q_min(nj), q_max(nj); //Create 2 array
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max); 

            //These 2 vectors will contain measure of data's read from senosor (trought subscriver)
            joint_positions_.resize(nj);  
            joint_velocities_.resize(nj);
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero(); 


            //(3a) Resize desired joint pos,vel,acc
            qd_.resize(nj); dqd_.resize(nj); ddqd_.resize(nj); qd_prev_.resize(nj); dqd_prev_.resize(nj);

            // Subscriber to jnt states. That is the implementation of the feedback
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
               rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
             //std::cout << "The initial EE pose is: " << std::endl;  
             //std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            //printFrame(init_cart_pose_);
             //std::cout << "The inverse kinematics returned: " <<std::endl; 
             //std::cout << q.data <<std::endl;

            /* DEBUG: check if the computed IK gives the original frame */ 
            /*
            KDL::Frame testFK;
            robot_->fkSol_->JntToCart(q,testFK);
            std::cout << "The associated direct kinematics returned: " <<std::endl; 
            std::cout << testFK <<std::endl;
            */

            // Initialize controller
            KDLController controller_(*robot_);
            

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1)); //In order to have e=/0 at starting point
            //std::cout<<"Got initial position"<<std::endl;
            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];


            //With 'traps' we mean trapezoidal velocity profile -> Require: _trajDuration, _accDuration.
            //With 'cubs' we mean that we're using a cubic poliomya for s -> Require: _trajDuration.

            // Plan trajectory
            double t = 0.0, traj_radius=0.37337;
            trajectory_point p;
            //To select the controller u have to modifie the controller section too
            switch(traj_sel_) {
                case 0:
                    //Using linear and cubs-------------
                    planner_ = KDLPlanner(traj_duration, init_position, end_position);
                    p = planner_.compute_trajectory_linear(t);
                    break;

                case 1:
                    
                    //Using Linear and traps ------------
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); 
                    p = planner_.compute_trajectory(t); //This return the point trought which the EE must pass at instant t.
                    break;

                case 2:
                    //Using Circular and Cubs--------------
                    planner_ = KDLPlanner(traj_duration, init_position, traj_radius);
                    p = planner_.compute_trajectory_circular(t);
                    break;

                case 3:
                    //Using Circular and Traps -------------
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, traj_radius);
                    p = planner_.compute_trajectory_circular(t, acc_duration);
                    break;
            }

            // Initial error
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;

            if(cmd_interface_ == "position"){
                // Create cmd_publisher
                //Send value on this topic implies to send comand to the robot
                //This is the topic for position publication
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                
                //std::cout << "First position commands sent: " << joint_positions_.data << std::endl;
                //(void) getchar();
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i); //fill comand with joint pos
                }
                //(3a)
            } else if(cmd_interface_ == "effort_joint"){
                // Create cmd_publisher
                //This is the topic for effort publication
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                // GET desired q
                KDL::Frame eeFrame_d(init_cart_pose_.M,toKDL(p.pos));
                //printFrame(eeFrame_d);
                robot_->getInverseKinematics(eeFrame_d, qd_);
                qd_prev_ = qd_;
                //std::cout << "computed q: "; printJntArray(qd_, nj); std::cout << std::endl;

                // GET desired qdot
                KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero()); //??? We initialize a Twist variable for vel IK
                //std::cout << "p.vel: "; printEigenVector3d(p.vel); std::cout << std::endl;
                robot_->getInverseKinematicsVel(eeVelTwist_d, dqd_);
                dqd_prev_ = dqd_;
                //std::cout << "computed qdot: "; printJntArray(dqd_, nj); std::cout << std::endl;
                
                // GET desired qddot
                ddqd_.data.setZero();
                //std::cout << "computed qddot: "; printJntArray(ddqd_, nj); std::cout << std::endl;

                // Apply controller
                desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
                /*
                //Initially commands are equal to 
                std::cout << "Initial commands: " << std::endl;
                for(int i=0; i<7; i++)
                        std::cout << desired_commands_[i] << " ";
                    std::cout << std::endl;
                */ 
            } 
            else if(cmd_interface_ == "effort_operational") {
                // Create cmd publisher
                //This is the topic for effort publication
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));

                //(4a) NVERSE DYNAMIC CONTROLLER IN OPERATIONAL SPACE 
                KDL::Frame eeFrame_d(init_cart_pose_.M,toKDL(p.pos));
                KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero());
                KDL::Twist eeAccTwist_d(toKDL(p.acc),KDL::Vector::Zero());
                
                // Apply controller
                //4a
                desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, eeAccTwist_d, Kp_j, Kpo_j,Kd_j,Kdo_j));

                
                /* std::cout << "Initial commands: " << std::endl;
                for(int i=0; i<7; i++)
                        std::cout << desired_commands_[i] << " ";
                    std::cout << std::endl; */
                
            }
            else if(cmd_interface_ == "velocity") {
                // Create cmd_publisher
                //This is the topic for velocity publication
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i); //fill comand with joint vel
                }
            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            
        }

        
    private:

        void cmd_publisher(){
            // Fissiamo: freq. <-> dt, durata totale
            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time = traj_duration; // (s)

             t_+=dt;

            // (3a) Initialize controller
            KDLController controller_(*robot_);

            if (t_ < total_time){

                trajectory_point p;
                // Retrieve the trajectory point
                switch(traj_sel_) {
                    case 0:
                        // Using linear and cubs-------------
                        p = planner_.compute_trajectory_linear(t_);
                        break;

                    case 1:
                        //2: Using linear and cubs-------------
                        p = planner_.compute_trajectory(t_);
                        break;

                    case 2:
                        //3: Using Circular and Cubs--------------
                        p = planner_.compute_trajectory_circular(t_);
                        break;

                    case 3:
                        //4: Using Circular and Traps -------------
                        p = planner_.compute_trajectory_circular(t_,acc_duration); 
                        break;
                }

                // Compute current KDLrobot EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.M = init_cart_pose_.M; desFrame.p = toKDL(p.pos); 

                //NEW CODE FROM HERE (version (2))
                // compute errors: linear and orientation
                //This error is the difference beetween the EE pos u should be (evaluated from s) in time instant t -
                //minus the actual EE position.
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data)); 
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "\nError norm at iteration " << iteration_ << " is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    //In prev version we just gave the last position instantanly
                    //Now we want a smooth control based on a FF of velocity + an error scaled (by 1, replace) 
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // NOOB Controller: desired trajectory is imposed 
                    nextFrame.p = toKDL(p.pos);

                    // Compute IK of the new frame we want to reach in order to have a smooth movement
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);

                    // Prepare for sending joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                //(3a)
                else if(cmd_interface_ == "effort_joint"){
                    
                    // GET desired q
                    KDL::Frame eeFrame_d(init_cart_pose_.M,toKDL(p.pos));
                    // Prof solution
                    //robot_->getInverseKinematics(eeFrame_d, qd_);
                    // Our trial
                    int ret = robot_->ikSol_->CartToJnt(qd_prev_,eeFrame_d,qd_);
                    if(ret != 0) {std::cout << robot_->ikSol_->strError(ret) << std::endl;};

                    // GET desired qdot
                    KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero()); //??? We initialize a Twist variable for vel IK
                    robot_->getInverseKinematicsVel(eeVelTwist_d, dqd_);

                    if(0) {
                        // DEBUG: check if the computed IK gives the original frame
                        KDL::Frame testFK;
                        robot_->fkSol_->JntToCart(qd_,testFK);
                        std::cout << "Desired position p.pos: \n" << eeFrame_d <<std::endl; 
                        std::cout << "The associated direct kinematics returned: " <<std::endl; 
                        std::cout << testFK <<std::endl;
                        //(void) getchar();

                        //DEBUG: check if the computed IK gives the original frame
                        KDL::FrameVel testFK_Vel;
                        KDL::JntArrayVel vvalues(qd_, dqd_);
                        robot_->fkVelSol_->JntToCart(vvalues,testFK_Vel);
                        std::cout << "\n\nDesired velocity p.vel: \n" << eeVelTwist_d.vel <<std::endl; 
                        std::cout << "The associated direct kinematics returned: " <<std::endl; 
                        std::cout << testFK_Vel.p.v <<std::endl;
                        (void) getchar();
                    }
                    

                    // GET desired qddot
                    //ddqd_.data = (dqd_.data - joint_velocities_.data)/dt; //Sol. 1
                    ddqd_.data = (dqd_.data - dqd_prev_.data)/dt;           //Sol. 2
                    dqd_prev_ = dqd_;
                    //ddqd_.data.setZero();

                    // Apply controller
                    desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
                    if(0) {
                        std::cout << "Commands: " << std::endl;
                        for(int i=0; i<7; i++)
                            std::cout << desired_commands_[i] << " ";
                        std::cout << std::endl;
                    }
                    
                    
                    if(0) {
                        Eigen::VectorXd gravityyy = robot_->getGravity();
                        std::cout << "GRAVITY VALUES" << std::endl;
                        for(int i=0; i<7; i++)
                            std::cout << gravityyy(i) << " ";
                        std::cout << std::endl;
                    }
                    
                }
                else if(cmd_interface_ == "effort_operational") {
                    //(4A) INVERSE DYNAMIC CONTROLLER IN OPERATIONAL SPACE
                    KDL::Frame eeFrame_d = desFrame;
                    KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero());
                    KDL::Twist eeAccTwist_d(toKDL(p.acc),KDL::Vector::Zero());

                    // Apply controller
                    desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, eeAccTwist_d, Kp_j, Kpo_j,Kd_j,Kdo_j));
                    
                    std::cout << "Commands: " << std::endl << std::endl;
                    for(int i=0; i<7; i++)
                        std::cout << desired_commands_[i] << " ";
                    std::cout << std::endl;
                    
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error; //also add orientation error
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_cmd_.data = joint_positions_.data + joint_velocities_cmd_.data*dt;

                    // Conversion between KDL::JntArray commands to std::vector commands
                    // Prepare for sending joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data)); 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            // At the end, maintains the robot in the final position
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

                trajectory_point p;
                // Retrieve the trajectory point
                switch(traj_sel_) {
                    case 0:
                        // Using linear and cubs-------------
                        p = planner_.compute_trajectory_linear(total_time);
                        break;
                    case 1:
                        //2: Using linear and cubs-------------
                        p = planner_.compute_trajectory(total_time);
                        break;
                    case 2:
                        //3: Using Circular and Cubs--------------
                        p = planner_.compute_trajectory_circular(total_time);
                        break;
                    case 3:
                        //4: Using Circular and Traps -------------
                        p = planner_.compute_trajectory_circular(total_time,acc_duration); 
                        break;
                }

                // Compute current KDLrobot EE frame
                KDL::Frame cartpos = robot_->getEEFrame();
                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p.pos); 

                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data)); 
                std::cout << "\nError norm at iteration " << iteration_ << " is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    // NOOB Controller: desired trajectory is imposed 
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = toKDL(p.pos);

                    // Compute IK of the new frame we want to reach in order to have a smooth movement
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                } 
                else if(cmd_interface_ == "effort_joint") {
                    
                    //std::cout << "Last position " << p.pos << "\n";
                    KDL::Frame eeFrame_d(init_cart_pose_.M,toKDL(p.pos));

                    int ret = robot_->ikSol_->CartToJnt(qd_prev_,eeFrame_d,qd_);
                    if(ret != 0) {std::cout << robot_->ikSol_->strError(ret) << std::endl;};

                    // GET desired qdot
                    dqd_.data.setZero();
                    // GET desired qddot
                    ddqd_.data.setZero();

                    // Apply controller
                    desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
                    if(1) {
                        std::cout << "Final Commands: " << std::endl;
                        for(int i=0; i<7; i++)
                            std::cout << desired_commands_[i] << " ";
                        std::cout << std::endl;
                    }
        
                } 
                else if(cmd_interface_ == "effort_operational") {
                    //(4A) INVERSE DYNAMIC CONTROLLER IN OPERATIONAL SPACE
                    // If you want to use the OpSpace Cotroller must uncomment from this line to after desired_commands_
                    KDL::Frame eeFrame_d = desFrame;
                    KDL::Twist eeVelTwist_d(KDL::Vector::Zero(),KDL::Vector::Zero());
                    KDL::Twist eeAccTwist_d(KDL::Vector::Zero(),KDL::Vector::Zero());

                    //SWITCHING CONTROLLER 
                    robot_->getInverseKinematics(desFrame, qd_);

                    // GET desired qdot
                    dqd_.data.setZero();
                    // GET desired qddot
                    ddqd_.data.setZero();
                    desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
              
                    std::cout << "Final Commands: " << std::endl << std::endl;
                    for(int i=0; i<7; i++)
                        std::cout << desired_commands_[i] << " "; 
                    std::cout << std::endl;
                }
                else if(cmd_interface_ == "velocity") {
                
                }
                
                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Create msg and publish (returns robot to vertical position)
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

            }
        }

        // Subscriber only takes values of joint pos and vels. It does not update the robot_ state
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_, joint_velocities_cmd_, joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        int traj_sel_;
        KDL::Frame init_cart_pose_;

        // OUR Attributes
        const double Kp_ = 100;
        const double Kd_ = 20;

        const double Kp_j = 30;
        const double Kd_j = 15;
        const double Kpo_j = 1;
        const double Kdo_j = 1;
        KDL::JntArray qd_, dqd_, ddqd_, qd_prev_, dqd_prev_;
        
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>()); //dynamic allocation of istance of the class
    rclcpp::shutdown();
    return 1;
}
