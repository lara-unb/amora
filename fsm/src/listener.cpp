#include "ros/ros.h"
#include "std_msgs/String.h"

#include "continuous.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Publisher ear_pub = n.advertise<std_msgs::String>("ear", 1000);

	ros::Rate loop_rate(10);
	
	decoder decoder;
	
	while(ros::ok())
	{
		//Message
		std_msgs::String msg;
		string speech;
		
		msg.data = decoder.recognize();
		
		ear_pub.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}

	return 0;
}
