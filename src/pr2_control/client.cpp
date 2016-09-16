#include <pr2_control/client.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iai_naive_object_db_client_main");
  ros::NodeHandle nh;
  pr2_control::ObjectDBClient my_object_db_client(nh);

  try
  {
    my_object_db_client.start();
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 0;
  }
  
  ros::spinOnce();

  return 0;
}
