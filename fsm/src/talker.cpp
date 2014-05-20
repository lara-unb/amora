#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "festival/festival.h"

using namespace std;

void Utterance(const std_msgs::String::ConstPtr& msg)
{
	
	festival_say_text(EST_String(msg->data.c_str()));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	
	ros::NodeHandle n;
	
	ros::Subscriber mouth_sub = n.subscribe("mouth", 1000, Utterance);
	ros::Publisher feedback = n.advertise<std_msgs::Bool>("feedback_mouth", 1000);
	
	std_msgs::Bool ack;
	ack.data = true;
	int heap_size = FESTIVAL_HEAP_SIZE;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded

	festival_initialize(load_init_files,heap_size);
	
	ros::Rate loop_rate(10);
	
	
	while(ros::ok())
	{
		feedback.publish(ack);
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
