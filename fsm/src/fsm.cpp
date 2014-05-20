#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "class_fsm.h"

using namespace std;

FSM fsm;
std_msgs::Bool feedback;
std_msgs::String msg;

void Synapse(const std_msgs::String::ConstPtr& speech)
{
	fsm.input(speech);
	//msg.data = speech->data;
	msg = fsm.output();
}

void Wait_speech_end(const std_msgs::Bool::ConstPtr& ack)
{
	feedback.data = ack->data;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "fsm");
	ros::NodeHandle n;
	
	ros::Publisher head_pub = n.advertise<std_msgs::String>("mouth", 1000);
	ros::Publisher m_state = n.advertise<std_msgs::String>("state_fsm", 1000);
	ros::Subscriber head_sub = n.subscribe("ear", 1000, Synapse);		
	ros::Subscriber feedback_mouth = n.subscribe("feedback_mouth", 1000, Wait_speech_end);
	ros::Rate loop_rate(10);
	
	std_msgs::String msg_state;
	
	msg = fsm.output();
	head_pub.publish(msg);
	cout << msg.data << endl;
	ros::spinOnce();
	
	while(ros::ok())
	{
		if(!msg.data.empty())
		{
			if(feedback.data)	head_pub.publish(msg);
			
		}
		msg_state.data = fsm.getState();
		m_state.publish(msg_state);
		msg.data.clear();
		//cout << feedback.data << endl;
		feedback.data = false;
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
