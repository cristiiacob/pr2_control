#include <pr2_control/l_arm_controller.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "l_arm_controller_node");
	ros::NodeHandle nh;

	pr2_control::RobotLArm my_arm(nh);

	try
        {
     	   	my_arm.start();
        }
        catch (const std::exception& ex)
        {
       		ROS_ERROR("%s",ex.what());
		return 0;
        }

	ros::spin();
/*	while (!arm.getState().isDone() && ros::ok())
	{
		usleep(50000);
	}
*/
}
