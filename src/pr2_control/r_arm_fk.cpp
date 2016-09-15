#include <pr2_control/r_arm_fk.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"r_arm_fk_node");
    	ros::NodeHandle nh;
	pr2_control::Transform_r_fk my_transform(nh);

	try
        {
     	   	my_transform.start();
        }
        catch (const std::exception& ex)
        {
       		ROS_ERROR("%s",ex.what());
		return 0;
        }

	ros::spin();

    return 0;
}

