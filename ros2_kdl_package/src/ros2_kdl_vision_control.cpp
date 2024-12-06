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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>  //Several try...
#include <geometry_msgs/msg/pose_stamped.hpp>

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
            declare_parameter("cmd_interface", "velocity"); // defaults to "velocity". That's where we choose type of control
            declare_parameter<int>("traj_sel", 0); // defaults to 0
            declare_parameter("task", "positioning"); // defaults to "positioning"
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("traj_sel", traj_sel_);
            get_parameter("task", task_);
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
            if (!(task_ == "positioning" || task_ == "look-at-point" || task_ == "merge_controllers"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }

            declare_parameter<double>("Kp", 1.0);
            declare_parameter<double>("Kd", 1.0);
            declare_parameter<double>("Kpo", 1.0);
            declare_parameter<double>("Kdo", 1.0);
            get_parameter("Kp", Kp_j);
            get_parameter("Kd", Kd_j);
            get_parameter("Kpo", Kpo_j);
            get_parameter("Kdo", Kdo_j);
            t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_available_ = false; 

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

            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            // Wait for the joint_state topic
            while(!joint_state_available_){
               rclcpp::spin_some(node_handle_);
            }

            std::cout << "Nr of joints: " << nj << " Nr of segments: " << robot_->getNrSgmts() << std::endl;
            KDL::Frame EEframe = robot_->getEEFrame();
            for(int i=0; i<robot_->getNrSgmts(); i++)
                std::cout << robot_->chain_.segments[i].getName() << std::endl;
            //FROM HERE MERGING....
            
            if(task_ == "positioning"){

                //Adapting tool_camera to camera_optical frame:
                //(camera_optical as defined isnt include in robot_ tree even if its defined in URDF)

                //RPY angles in radians: this value are got from urdf (Cam-CamOptical relative orientation)
                double roll = -1.57;
                double pitch = 0.0; 
                double yaw = -1.57;  
                // Create a KDL::Rotation from RPY

                KDL::Rotation rotation_Cam_CamOptical = KDL::Rotation::RPY(roll, pitch, yaw);

                // Create a KDL::Vector for position (0, 0, 0)
                KDL::Vector position(0.0, 0.0, 0.0);

                // Create a KDL::Frame with the rotation and position
                KDL::Frame Cam_CamOptical(rotation_Cam_CamOptical, position);
                

                // Update KDLrobot object
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                KDL::Frame f_T_ee = Cam_CamOptical; //Now the last frame is oriented with z looking at aruco
                robot_->addEE(f_T_ee);
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));


                // Compute EE frame
                init_cart_pose_ = robot_->getEEFrame();
                 //std::cout << "The initial EE pose is: " << std::endl;  
                 //std::cout << init_cart_pose_ <<std::endl;

                // Initialize controller
                KDLController controller_(*robot_);                

                // EE's trajectory initial position (just an offset)
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
                std::cout<<"EE init position (wrt world frame): "<<init_position<<std::endl;
                //HW3:
                //EE final position must be Aruco_Tag_Position-Offset (expressed in base frame)
                //1: get aruco pos (wrt tool 0)
                // Subscriber to aruco_pos. Once
                ArucoPos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_position, this, std::placeholders::_1));
        
                //  Waiting 
                RCLCPP_INFO(this->get_logger(), "Aruco Pose not available ...");
                while(!aruco_pose_available_){
                    
                   rclcpp::spin_some(node_handle_);
                }
                //after this while u get the final desired position and orientation computed by the callback
                //end_position_wrt_world_frame HAS BEEN FILLED
                //We using the fact that the final position can be evaluated as EE_Pose (base frame) + Aruco_Pose (camera frame)
                Eigen::Vector3d end_position;
                // Convert KDL::Rotation to Eigen::Matrix3d
            
                end_position<<
                end_position_wrt_world_frame[0],
                end_position_wrt_world_frame[1],
                end_position_wrt_world_frame[2];

                std::cout<<"Computed end position (wrt world frame): "<<end_position<<std::endl;

                
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

                // Create cmd_publisher_positioning
                //This is the topic for velocity publication
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher_positioning, this));
                
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i); //fill comand with joint vel
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            } //Closing if task==positioning

            else if(task_=="look-at-point"){
                // Update KDLrobot object
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                KDL::Frame f_T_ee = KDL::Frame::Identity();
                robot_->addEE(f_T_ee);
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                double xx,yy,zz,ww;
                EEframe.M.GetQuaternion(xx,yy,zz,ww);
                std::cout << xx << " " << yy << " " << zz << " " << ww << "\n"; 
                (void) getchar(); //DEBUG

                // Initialize controller
                KDLController controller_(*robot_);
                
                // Subscriber to jnt states. That is the implementation of the feedback
                arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                    "/aruco_single/position", 10, std::bind(&Iiwa_pub_sub::aruco_subscriber, this, std::placeholders::_1));

                // Wait for the joint_state topic
                while(!aruco_pose_available_){
                    RCLCPP_INFO(this->get_logger(), "No ARUCO data received yet! ...");
                   rclcpp::spin_some(node_handle_);
                }

                if(cmd_interface_ == "effort_joint"){
                    // Create cmd_publisher_look_at_point
                    //This is the topic for effort publication
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher_look_at_point, this));

                    // GET desired qdot
                    //LOOK-AT-POINT CONTROLLER
                    dqd_prev_ = joint_velocities_;
                    dqd_.data = controller_.look_at_point_control(cPo);
                    // GET desired q
                    qd_prev_ = joint_positions_;
                    qd_.data = qd_prev_.data + dqd_.data*dt;
                    // GET desired qddot
                    ddqd_.data.setZero();
                    //std::cout << "computed qddot: "; printJntArray(ddqd_, nj); std::cout << std::endl;

                    // Apply controller
                    desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
                } 
                else if(cmd_interface_ == "effort_operational") {
                    // Create cmd publisher
                    //This is the topic for effort publication
                    cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher_look_at_point, this));

                    // GET desired qdot
                    //LOOK-AT-POINT CONTROLLER
                    dqd_prev_ = joint_velocities_;
                    dqd_.data = controller_.look_at_point_control(cPo);
                    // GET desired q
                    qd_prev_ = joint_positions_;
                    qd_.data = qd_prev_.data + dqd_.data*dt;
                    // GET desired qddot
                    //ddqd_.data.setZero();
                    ddqd_.data = (dqd_.data - dqd_prev_.data)/dt;

                    // Wrap q_des and dq_des in JntArrayVel structure
                    KDL::JntArrayVel desJntPosVel(qd_, dqd_);
                    // Define FrameVel structure for storing forward kinematics results and compute FK
                    KDL::FrameVel desCartPosVel; robot_->getDirectKinematicsPosVel(desJntPosVel, desCartPosVel);
                    
                    // Extract desired pose
                    KDL::Frame eeFrame_d(desCartPosVel.M.R, desCartPosVel.p.p);
                    // Extract desired velocity
                    KDL::Twist eeVelTwist_d(desCartPosVel.p.v, desCartPosVel.M.w);
                    // Define acceleration structure and compute 2nd FK
                    //KDL::Twist eeAccTwist_d = KDL::Twist::Zero();

                    // Define acceleration structure and compute 2nd FK
                    //using angle and axis we need the geometric jacobian, so vdot = J qdd + jdot qdot
                    //Define J_dot*q_dot
                    Eigen::Matrix<double,6,1> Jdotqdot = robot_->getEEJacDotqDot();
                    KDL::Twist Ades( toKDLTwist( (robot_->getEEJacobian().data) * ddqd_.data + Jdotqdot)) ;
                    
                    // Apply Op. Space Controller
                    desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, Ades, Kp_j, Kpo_j,Kd_j,Kdo_j));
                }
                else if(cmd_interface_ == "velocity") {
                    // Create cmd_publisher_look_at_point
                    //This is the topic for velocity publication
                    cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher_look_at_point, this));
                
                    //LOOK-AT-POINT CONTROLLER
                    joint_velocities_cmd_.data = controller_.look_at_point_control(cPo);

                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            //Closing if(task_=="look-at-point")
            } else {
                //RPY angles in radians: this value are got from urdf (Cam-CamOptical relative orientation)
                double roll = -1.57;
                double pitch = 0.0; 
                double yaw = -1.57;  
                // Create a KDL::Rotation from RPY
                KDL::Rotation rotation_Cam_CamOptical = KDL::Rotation::RPY(roll, pitch, yaw);
                // Create a KDL::Vector for position (0, 0, 0)
                KDL::Vector position(0.0, 0.0, 0.0);
                // Create a KDL::Frame with the rotation and position
                KDL::Frame Cam_CamOptical(rotation_Cam_CamOptical, position);
                // Update KDLrobot object
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
                KDL::Frame f_T_ee = Cam_CamOptical; //Now the last frame is oriented with z looking at aruco
                robot_->addEE(f_T_ee);
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // Compute EE frame
                init_cart_pose_ = robot_->getEEFrame();

                //Define Controller object
                KDLController controller_(*robot_);

                //ARUCO
                // Subscriber to jnt states. That is the implementation of the feedback
                arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                    "/aruco_single/position", 10, std::bind(&Iiwa_pub_sub::aruco_subscriber, this, std::placeholders::_1));
                // Wait for the joint_state topic
                while(!aruco_pose_available_){
                    RCLCPP_INFO(this->get_logger(), "No ARUCO data received yet! ...");
                   rclcpp::spin_some(node_handle_);
                }

                //PUBLISHER
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(dt*1000)), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher_merge_controllers, this));

                //PLANNER CONSTRUCTOR
                // EE's trajectory initial and end position
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));
                Eigen::Vector3d end_position(init_position + Eigen::Vector3d(0.0,0.0,0.2));
                
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

                // REFERENCES
                KDL::Frame eeFrame_d(KDL::Rotation::Identity(),toKDL(p.pos));
                KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero());
                KDL::Twist eeAccTwist_d(toKDL(p.acc),KDL::Vector::Zero());
                // Compute Desired Frame
                Eigen::Matrix<double,3,1> aruco_pos_n = cPo;//(aruco_pose[0],aruco_pose[1],aruco_pose[2]);           //HOMEWORK3 cPo. ottengo il VETTORE posizione (Matrix<double,3,1>) dell'aruco marker rispetto al camera frame: è il vettore che collega l'origine dell'aruco marker frame al camera frame. Lo converto a un elemento di classe Eigen::Matrix perché solo così posso usarlo nelle elaborazioni matematiche 
                aruco_pos_n.normalize();                                                                                                 //HOMEWORK3 versore s. Il versore che vogliamo eguagliare durante la rotazione. Ottenuto per normalizzazione di cPo == aruco_pos_n. ATTENZIONE: questa cosa normalizza il VETTORE ORIGINALE aruco_pos_n, che quindi ora è == s
                Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;                                                          //HOMEWORK3 vettore r_o = cross product/prodotto vettoriale tra versore zc (asse z della camera) e  versore s. Nell'esempio "r_o" = "a" è un versore ortogonale al piano identificato da zc ed s.
                double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));                                                 //HOMEWORK3 angolo tra i versori zc e s, computato dall'arcoseno del dot product/prodotto scalare tra i due.
                KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);                                 //HOMEWORK3 matrice di rotazione "RotationError" (usando la rappresentazione "Asse Angolo":: r_o = asse, aruco_angle = angolo) tra la rotazione attuale della camera attorno all'asse r_o e la rotazione desiderata attorno allo stesso asse per allinearsi con s. E' quindi un errore rispetto al CAMERA frame (se vuoi la desired matrix rispetto al base frame, devi moltiplicare la desired matrix per la trasformazione tra camera frame e base frame fornita da "getEEFrame()"           
                eeFrame_d.M = robot_->getEEFrame().M*Re;                
                
                // APPLY OP. SPACE CONTROLLER
                desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, eeAccTwist_d, 80.0, 40.0, 40.0, 12.6));
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            }
        }

    private:
        // Subscriber only takes values of joint pos and vels. It does not update the robot_ state
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        void cmd_publisher_merge_controllers() {
            // Initialize controller
            KDLController controller_(*robot_);

            // define trajectory
            double total_time = traj_duration; // (s)
            t_+=dt;
            trajectory_point p;

            if (t_ < total_time){
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
                // REFERENCES
                KDL::Frame eeFrame_d(KDL::Rotation::Identity(),toKDL(p.pos));
                KDL::Twist eeVelTwist_d(toKDL(p.vel),KDL::Vector::Zero());
                KDL::Twist eeAccTwist_d(toKDL(p.acc),KDL::Vector::Zero());
                // Compute Desired Frame
                Eigen::Matrix<double,3,1> aruco_pos_n = cPo;                                                                             //HOMEWORK3 cPo. ottengo il VETTORE posizione (Matrix<double,3,1>) dell'aruco marker rispetto al camera frame: è il vettore che collega l'origine dell'aruco marker frame al camera frame. Lo converto a un elemento di classe Eigen::Matrix perché solo così posso usarlo nelle elaborazioni matematiche 
                aruco_pos_n.normalize();                                                                                                 //HOMEWORK3 versore s. Il versore che vogliamo eguagliare durante la rotazione. Ottenuto per normalizzazione di cPo == aruco_pos_n. ATTENZIONE: questa cosa normalizza il VETTORE ORIGINALE aruco_pos_n, che quindi ora è == s
                Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;                                                          //HOMEWORK3 vettore r_o = cross product/prodotto vettoriale tra versore zc (asse z della camera) e  versore s. Nell'esempio "r_o" = "a" è un versore ortogonale al piano identificato da zc ed s.
                double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));                                                 //HOMEWORK3 angolo tra i versori zc e s, computato dall'arcoseno del dot product/prodotto scalare tra i due.
                KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);                                 //HOMEWORK3 matrice di rotazione "RotationError" (usando la rappresentazione "Asse Angolo":: r_o = asse, aruco_angle = angolo) tra la rotazione attuale della camera attorno all'asse r_o e la rotazione desiderata attorno allo stesso asse per allinearsi con s. E' quindi un errore rispetto al CAMERA frame (se vuoi la desired matrix rispetto al base frame, devi moltiplicare la desired matrix per la trasformazione tra camera frame e base frame fornita da "getEEFrame()"           
                eeFrame_d.M = robot_->getEEFrame().M*Re;                

                // APPLY OP. SPACE CONTROLLER
                desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, eeAccTwist_d, 80.0, 40.0, 40.0, 12.6));

            }
            // At the end, maintains the robot in the final position
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
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

                // REFERENCES
                KDL::Frame eeFrame_d(KDL::Rotation::Identity(),toKDL(p.pos));
                KDL::Twist eeVelTwist_d(KDL::Vector::Zero(),KDL::Vector::Zero());
                KDL::Twist eeAccTwist_d(KDL::Vector::Zero(),KDL::Vector::Zero());

                // APPLY OP. SPACE CONTROLLER
                desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, eeAccTwist_d, Kp_j, Kpo_j,Kd_j,Kdo_j));

                //SWITCHING CONTROLLER 
                robot_->getInverseKinematics(eeFrame_d, qd_);

                // GET desired qdot
                dqd_.data.setZero();
                // GET desired qddot
                ddqd_.data.setZero();
                desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        //HW3: 
        //POSITIONING FUNCTIONS
        void cmd_publisher_positioning(){

            // define trajectory
            double total_time = traj_duration; // (s)

            t_+=dt;
            trajectory_point p;

            if (t_ < total_time){
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
                //HW3: we need to feel in defFrame the orientation evaluated in the aruco's callback 
                KDL::Frame desFrame;
                desFrame.M = end_orientation_wrt_world_frame; 
                desFrame.p = toKDL(p.pos); 

                //NEW CODE FROM HERE (version (2))
                // compute errors: linear and orientation
                //This error is the difference beetween the EE pos u should be (evaluated from s) in time instant t -
                //minus the actual EE position.
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data)); 
                Eigen::Vector3d o_error = computeOrientationError(toEigen(desFrame.M ), toEigen(cartpos.M));
                
                // Compute differential IK (CONTROLLER)
                Vector6d cartvel; cartvel << p.vel + 5*error, o_error; //also add orientation error
                joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                joint_positions_cmd_.data = joint_positions_.data + joint_velocities_cmd_.data*dt;

                // Conversion between KDL::JntArray commands to std::vector commands
                // Prepare for sending joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                } 
            }
            // At the end, maintains the robot in the final position
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0;
                } 

            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            // Create msg and publish (returns robot to vertical position)
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        //Callback to retrive aruco position
        void aruco_position(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {   
            if(!aruco_pose_available_){
            if(1){
            // Retrieve position DEBUG
            RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]",
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            // Retrieve orientation (as quaternion) DEBUG
            RCLCPP_INFO(this->get_logger(), "Orientation: [x: %f, y: %f, z: %f, w: %f]",
                        msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w);
            }

            

            tf2::Quaternion aruco_quat(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);


            //Retrive Offset from Keyb:
            std::vector<float> input_pose(6);

            std::cout << "Enter position offset (x, y, z): ";
            for (int i = 0; i < 3; ++i)
            {
                std::cin >> input_pose[i];
            }

            std::cout << "Enter orientation offset (roll, pitch, yaw in radians): ";
            for (int i = 3; i < 6; ++i)
            {
                std::cin >> input_pose[i];
            }

            if(0){
            std::cout << "You entered the following offsets:\n";
            std::cout << "Position: [" << input_pose[0] << ", " << input_pose[1] << ", " << input_pose[2] << "]\n";
            std::cout << "Orientation (Euler angles): ["
                      << input_pose[3] << " (roll), "
                      << input_pose[4] << " (pitch), "
                      << input_pose[5] << " (yaw)]\n";
            }

             // Step 1: Final orientation
            //Convert user input (Euler angles) to a quaternion
            tf2::Quaternion input_quat; //Contains offset in quaternion
            input_quat.setRPY(input_pose[3], input_pose[4], input_pose[5]); 

            //Resulting final orientation:
             aruco_quat.normalize();
             if(0){
                std::cout << "Final quaternion matrix" << std::endl;
                std::cout << "  x: " << aruco_quat.x() << std::endl;
                std::cout << "  y: " << aruco_quat.y() << std::endl;
                std::cout << "  z: " << aruco_quat.z() << std::endl;
                std::cout << "  w: " << aruco_quat.w() << std::endl;
            }

            //Orientation
            KDL::Rotation input_rot_matrix;
            input_quat.normalize();
            input_rot_matrix = KDL::Rotation::Quaternion(input_quat.x(),
                                                        input_quat.y(), 
                                                        input_quat.z(), 
                                                        input_quat.w());

            KDL::Rotation aruco_rot_matrix;
            aruco_quat.normalize();
            aruco_rot_matrix = KDL::Rotation::Quaternion(aruco_quat.x(),
                                                        aruco_quat.y(), 
                                                        aruco_quat.z(), 
                                                        aruco_quat.w());    

            end_orientation_wrt_world_frame = aruco_rot_matrix*input_rot_matrix; 

             std::cout <<"end_orient with method 2:"<<std::endl;
             for (int i = 0; i < 9; ++i)
            {
                std::cout << end_orientation_wrt_world_frame.data[i];
            }
            std::cout<<std::endl;

            //Express input_pose in camera ref
            // Create an KDL Vect from the std::vector
            KDL::Vector input_pose_kdl(input_pose[0], input_pose[1], input_pose[2]);
            KDL::Vector input_pose_wrt_world_frame;

            input_pose_wrt_world_frame=aruco_rot_matrix*input_pose_kdl;

            std::cout<<"Input pose wrt world frame:  "<<std::endl<<input_pose_wrt_world_frame<<std::endl;

            //Step 2: Final position
            end_position_wrt_world_frame<<
                msg->pose.position.x + input_pose_wrt_world_frame[0],
                msg->pose.position.y + input_pose_wrt_world_frame[1],
                msg->pose.position.z + input_pose_wrt_world_frame[2];

          //   std::string reference_frame = msg->header.frame_id;
          //  RCLCPP_INFO(this->get_logger(), "Pose is referenced to frame: %s", reference_frame.c_str());

             aruco_pose_available_=true;
            //I want this to run only once
            
            }
        }

        //LOOK AT POINT FUNCTIONS:
        void cmd_publisher_look_at_point(){

            //Initialize controller
            KDLController controller_(*robot_);         

            if(cmd_interface_ == "effort_joint"){
                // GET desired qdot
                //LOOK-AT-POINT CONTROLLER
                dqd_prev_ = joint_velocities_;
                //dqd_prev_ = dqd_;
                dqd_.data = controller_.look_at_point_control(cPo);

                // GET desired q
                qd_prev_ = joint_positions_;
                //qd_prev_ = qd_;
                qd_.data = qd_prev_.data + dqd_.data*dt;
                
                // GET desired qddot
                ddqd_.data = (dqd_.data - dqd_prev_.data)/dt;

                // Apply controller
                desired_commands_ = toStdVector(controller_.idCntr(qd_, dqd_, ddqd_, Kp_, Kd_));                
                
            }
            else if(cmd_interface_ == "effort_operational") {
                // GET desired qdot
                //LOOK-AT-POINT CONTROLLER
                dqd_prev_ = joint_velocities_;
                dqd_.data = controller_.look_at_point_control(cPo,Eigen::VectorXd::Zero(7),1.5);
                // GET desired q
                qd_prev_ = joint_positions_;
                qd_.data = qd_prev_.data + dqd_.data*dt;
                // GET desired qddot
                //ddqd_.data.setZero();
                ddqd_.data = (dqd_.data - dqd_prev_.data)/dt;

                // Wrap q_des and dq_des in JntArrayVel structure
                KDL::JntArrayVel desJntPosVel(qd_, dqd_);
                // Define FrameVel structure for storing forward kinematics results and compute FK
                KDL::FrameVel desCartPosVel; robot_->getDirectKinematicsPosVel(desJntPosVel, desCartPosVel);

                
                // Extract desired pose
                KDL::Frame eeFrame_d(desCartPosVel.M.R, desCartPosVel.p.p);
                // Extract desired velocity
                KDL::Twist eeVelTwist_d(desCartPosVel.p.v, desCartPosVel.M.w);
                // Define acceleration structure and compute 2nd FK
                //KDL::Twist eeAccTwist_d = KDL::Twist::Zero();

                // Define acceleration structure and compute 2nd FK
                //using angle and axis we need the geometric jacobian, so vdot = J qdd + jdot qdot
                //Define J_dot*q_dot
                Eigen::Matrix<double,6,1> Jdotqdot = robot_->getEEJacDotqDot();
                KDL::Twist Ades( toKDLTwist( (robot_->getEEJacobian().data) * ddqd_.data + Jdotqdot)) ;
                
                // Apply Op. Space Controller
                desired_commands_ = toStdVector(controller_.idCntr(eeFrame_d, eeVelTwist_d, Ades, Kp_j, Kpo_j,Kd_j,Kdo_j));

            }
                else if(cmd_interface_ == "velocity") {
                    //LOOK-AT-POINT CONTROLLER
                    joint_velocities_cmd_.data = controller_.look_at_point_control(cPo,Eigen::VectorXd::Zero(7),1.5);

                    // Send joint velocity commands
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
        }

        // Subscriber takes values of aruco tag position wrt Camera Frame
        void aruco_subscriber(const geometry_msgs::msg::Vector3Stamped& sensor_msg){
            aruco_pose_available_ = true;
            cPo[0] = sensor_msg.vector.x;
            cPo[1] = sensor_msg.vector.y;
            cPo[2] = sensor_msg.vector.z;
            //std::cout << cPo << std::endl;

            //std::string reference_frame = sensor_msg.header.frame_id;
            //RCLCPP_INFO(this->get_logger(), "Pose is referenced to frame: %s", reference_frame.c_str());
        }


        //CLASS MEMBER
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_; //Can be both positioning or look_at_point comand
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_, joint_velocities_cmd_, joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        int traj_sel_;
        std::string task_;          //to select the task u want to do
        KDL::Frame init_cart_pose_;

        // OUR Attributes
        const double Kp_ = 100;
        const double Kd_ = 20;

        double Kp_j;// = 1;
        double Kd_j;// = 1;
        double Kpo_j;// = 1;
        double Kdo_j;// = 1;
        KDL::JntArray qd_, dqd_, ddqd_, qd_prev_, dqd_prev_;

        //HW3-------

        // LOOK AT POINT REQUIRED MEMBER OF CLASS------------

        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr arucoSubscriber_;
        Eigen::Vector3d cPo;


        //POSITIONING REQUIRED MEMBER OF CLASS---------------

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ArucoPos_;
        Eigen::Vector3d end_position_wrt_world_frame;
        KDL::Rotation end_orientation_wrt_world_frame;
        bool aruco_pose_available_;   

        //HW3-------    
};


int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>()); //dynamic allocation of istance of the class
    rclcpp::shutdown();
    return 1;
}
