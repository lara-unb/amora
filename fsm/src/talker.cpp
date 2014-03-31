#include "ros/ros.h"
#include "std_msgs/String.h"
#include "festival.h"

void Utterance(const std_msgs::String::ConstPtr& msg)
{
	festival_say_text(EST_String(msg->data.c_str()));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("mouth", 1000, Utterance);
	
	int heap_size = FESTIVAL_HEAP_SIZE;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded

	festival_initialize(load_init_files,heap_size);
	
	festival_say_text(EST_String("HI. I AM ARAMIS"));
	
	while(ros::ok())
	{
		ros::spinOnce();
	}
	
	return 0;
}
