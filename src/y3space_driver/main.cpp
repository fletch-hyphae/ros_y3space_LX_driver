#include <Y3SpaceDriver.h>


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "Y3SpaceDriver");
  	ros::NodeHandle nh;
  	ros::NodeHandle pnh("~");
  	Y3SpaceDriver driver(nh, pnh);
  	driver.run();
}
