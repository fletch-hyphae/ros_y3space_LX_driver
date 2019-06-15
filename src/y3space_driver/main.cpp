#include <y3space_driver/Y3SpaceDriver.h>


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "Y3SpaceDriver");
  	ros::NodeHandle nh;
  	ros::NodeHandle pnh("~");
  	YostLabDriver driver(nh, priv_nh);
  	driver.run();
}
