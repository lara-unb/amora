#include "ros/ros.h"
#include "std_msgs/String.h"
#include "festival.h"
#include "fsm.h"

typedef struct Buffer
{
	std_msgs::String string;
	ros::Time timestamp;
}msg_Buffer;

msg_Buffer buffer;
int state = 0;
bool state_wait = false;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	//cout << msg->data.c_str() << endl;
	buffer.string = *msg;
	buffer.timestamp = ros::Time::now();
	//state = 2;
	fsm::check(msg->data.c_str(), &state, &state_wait);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

   while (ros::ok())
   {
   	switch(state)
   	{
   		case 0:
   			fsm::init();
   			state++;
   			break;
   		case 1:
   			fsm::wait();
   			break;
   		case 2:
   			fsm::ack(buffer.timestamp, &state, &state_wait);
   			break;
   		case 3:
   			fsm::follow();
   			state = 1;
   			state_wait = false;
   			break;
   		case 4:
   			fsm::tour_guide();
   			state = 1;
   			state_wait = false;
   			break;
   	}
   	
   	ros::spinOnce();
   }


  return 0;
}

