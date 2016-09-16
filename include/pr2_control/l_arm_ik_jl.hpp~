#ifndef PR2_CONTROL_L_ARM_IK_JL_HPP
#define PR2_CONTROL_L_ARM_IK_JL_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <pr2_control/pr2_control_functions.hpp>
#include <kdl/frames_io.hpp>
#include <geometry_msgs/Pose.h>
#include "kdl_conversions/kdl_msg.h"
#include <std_msgs/Float32MultiArray.h>
#include <urdf/model.h>

namespace pr2_control
{
	class Transform_l_ik_jl
	{
	    public:
		Transform_l_ik_jl(const ros::NodeHandle& nh): nh_(nh), base_frame_("base_footprint"), target_frame_("r_gripper_tool_frame"), once_(true)
		{}

		~Transform_l_ik_jl() {}
		
		void stateCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
                     	last_joint_state_ = *msg;
		
			if(once_)
			{
				joint_indeces_ = createJointIndices(createIndexMap(msg, joint_names_), joint_names_);
				once_ = false;
			}
		
		}

		void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
		{	
// TODO: print input msg
			tf::poseMsgToKDL(*msg, goal_frame_);		
			if(!last_joint_state_.position.empty())
                        {	
				q_out_ = calculateIK_JL(ik_solver_pos_, goal_frame_, toKDL(last_joint_state_, joint_indeces_));
				ROS_INFO("%d", q_out_.rows());
			
				std_msgs::Float32MultiArray pub_msg;
				for(int i = 0; i< q_out_.rows(); ++i)
				{
					ROS_INFO("%f", q_out_(i));
					pub_msg.data.push_back(q_out_(i));
				
				}
				
				while(pub_.getNumSubscribers() == 0)
   					ros::Duration(1).sleep();
				
				pub_.publish(pub_msg);	
				ROS_INFO("Sent!");
                        }
			else
				ROS_WARN("Could not process goal because there was not JointStates, yet.");
		}
	
		void createChain(const std::string& robot_desc)
		{
			KDL::Tree my_tree;
		
			if (!kdl_parser::treeFromString(robot_desc, my_tree))
		        	throw std::runtime_error( "Could not construct the tree" );


			if (!my_tree.getChain(base_frame_, target_frame_, chain_))
		                throw std::runtime_error( "Could not construct the chian" );
		}

		void start()
		{			
		        std::string robot_desc;
			nh_.param("robot_description", robot_desc, std::string());
			createChain(robot_desc);
			urdf::Model model;

    			if (!model.initString(robot_desc))
			{
      				ROS_ERROR("Failed to parse urdf file");
       				//return -1; throw exception
     			}
			
			joint_names_ = getJointNames(chain_);
			
			for(size_t i = 0; i <joint_names_.size(); ++i)
				ROS_INFO("%s", joint_names_[i].c_str());

			KDL::JntArray q_min(chain_.getNrOfJoints()), q_max(chain_.getNrOfJoints());
			for (size_t i=0; i < chain_.getNrOfJoints(); ++i) 
			{
				if(model.getJoint(joint_names_[i])->type == 2) //CONTINUOUS
				{
					q_min(i) = -1000;
					q_max(i) = 1000;
				}
				else
				{
					q_min(i) = model.getJoint(joint_names_[i])->limits->lower;
	       				q_max(i) = model.getJoint(joint_names_[i])->limits->upper;	
				}
				ROS_INFO("%s lower: %f", joint_names_[i].c_str(), q_min(i));
				ROS_INFO("%s upper: %f", joint_names_[i].c_str(), q_max(i));
   			}
			fkpossolver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(KDL::ChainFkSolverPos_recursive(chain_));
			ikvelsolver_ = std::make_shared<KDL::ChainIkSolverVel_pinv_givens>(KDL::ChainIkSolverVel_pinv_givens(chain_));
			ik_solver_pos_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(KDL::ChainIkSolverPos_NR_JL(chain_, q_min, q_max, *fkpossolver_.get(), *ikvelsolver_.get()));	
			sub_state_ = nh_.subscribe("joint_states", 1, &Transform_l_ik_jl::stateCallback, this);
			
			try
			{
				ros::Duration(2).sleep();
			}			
			catch(std::exception& e)
			{
				ROS_ERROR("%s",e.what());	
			}
					
		 	sub_goal_ = nh_.subscribe("/tf_goal_l_arm", 1, &Transform_l_ik_jl::goalCallback, this);	
			pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/goal_joint_pos_l_arm", 1);
	    	}
	
	    private:
		ros::NodeHandle nh_;
		std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos_;
		std::shared_ptr<KDL::ChainFkSolverPos_recursive> fkpossolver_;
		std::shared_ptr<KDL::ChainIkSolverVel_pinv_givens> ikvelsolver_;
		std::string base_frame_;
		std::string target_frame_;
		bool once_;
		KDL::Chain chain_; 
		std::vector<size_t> joint_indeces_;
		std::vector<std::string> joint_names_;
		ros::Subscriber sub_state_;
		ros::Subscriber sub_goal_;
		ros::Publisher pub_;
		sensor_msgs::JointState last_joint_state_;
		KDL::JntArray q_out_;
		KDL::Frame goal_frame_;
	};
}

#endif
