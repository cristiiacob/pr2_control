#include <pr2_control/l_arm_ik_jl.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"l_arm_ik_jl_node");
    	ros::NodeHandle nh;
	pr2_control::Transform_l_ik_jl my_transform(nh);

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
