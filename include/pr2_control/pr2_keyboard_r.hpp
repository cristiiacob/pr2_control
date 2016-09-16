#ifndef PR2_CONTROL_PR2_KEYBOARD_R_HPP
#define PR2_CONTROL_PR2_KEYBOARD_R_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace pr2_control
{
	class TFGoal_R
	{
	    public:
		TFGoal_R(const ros::NodeHandle& nh, const geometry_msgs::Pose& tf_goal): nh_(nh), tf_goal_(tf_goal)
		{}

		~TFGoal_R() {}
			
	//	void callback(const ros::TimerEvent& e)
		void callback()
	        {
			pub_.publish(tf_goal_);
		}

		//void start(const ros::Duration& period)
		void start()
     		{
        		pub_ = nh_.advertise<geometry_msgs::Pose>("/tf_goal_r_arm", 1);
        	//	timer_ = nh_.createTimer(period, &TFGoal_R::callback, this);
			while(pub_.getNumSubscribers() == 0)
   				 ros::Duration(0.5).sleep();	
			callback();
      		}
  		
	    private:	
		ros::NodeHandle nh_;
      		ros::Publisher pub_;
      	//	ros::Timer timer_;
		geometry_msgs::Pose tf_goal_;

	};	
}

#endif
