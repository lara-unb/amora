#include "class_fsm.h"


FSM::FSM()
{
	state_wait = false;
	state = INIT;
	machine();
	filter = filter->getInstance();
}

void FSM::machine()
{
	switch(state)
	{
		case INIT:
			speech = "GOOD MORNING. MY NAME IS ARAMIS AND I WILL BE YOUR TOUR GUIDE.";
			state++;
			break;
		case WAIT:
			//wait();
			break;
		case ACK:
			ack();
			break;
		case FOLLOW:
			follow();
			state = WAIT;
			break;
		case TOUR_GUIDE:
			tour_guide();
			state = WAIT;
			break;
		case SELF_DESTR:
			selfdestr();
			state = WAIT;
			break;
	}
}

void FSM::follow()
{

}

void FSM::tour_guide()
{

}

void FSM::selfdestr()
{
	speech = std::string("SELF DESTRUCTION MODE ACTIVADED. ") + std::string("SELF DESTRUCTION START IN 5 SECONDS. ") + std::string("5 SECONDS.");
	//system("sleep 5s; shutdown -h now")
}
/*
int FSM::getState()
{
	return state;
}
*/
void FSM::ack()
{
	//static bool first = true;
	
	//if(first)	output(speech);
	ros::Time now = ros::Time::now();
	ros::Duration time;
	
	time = now - buffer.timestamp;
	ros::Duration ten_seconds(10.0);
	
	//first = false;
	if(time > ten_seconds)
	{
		state = ACK;
		state_wait = false;
		speech.clear();
		//first = true;
	}
}

std::string FSM::getState()
{
	switch(state)
	{
		case INIT:
			return std::string("init");
		case WAIT:
			return std::string("wait");
		case ACK:
			return std::string("ack");
		case FOLLOW:
			return std::string("follow");

		case TOUR_GUIDE:
			return std::string("tour_guide");
		
		case SELF_DESTR:
			return std::string("self_destr");
	}
}

void FSM::input(const std_msgs::String::ConstPtr& msg)
{
	speech = filter->answer(msg->data);
	if(speech == "YES MASTER")
	{
		buffer.str = *msg;
		buffer.timestamp = ros::Time::now();
	}
	this->whatstate();
	this->machine();
}

void FSM::whatstate()
{
	if(speech == "YES MASTER")
	{
		state_wait = true;
		state = ACK;
		return;
	}
	if(state_wait)
	{
		if(speech == "I WILL FOLLOW YOU NOW")	state = FOLLOW;
		else if(speech == "I WILL BE TOUR GUIDE")	state = TOUR_GUIDE;
		else if(speech == "I WILL BE TOUR GUIDE")	state = TOUR_GUIDE;
		else if(speech ==  "SELF DESTRUCTION")	state = SELF_DESTR;
		this->ack();
	}
}

std_msgs::String FSM::output()
{
	std_msgs::String msg;
	
	//std::cout << speech << std::endl;
	msg.data = speech;
	speech.clear();
	
	return msg;
}
/*
std_msgs::String FSM::output()
{
	string strn(str);
	std_msgs::String msg;
	
	msg.data = str;
	
	return msg;
}
*/
FSM::~FSM()
{
//des
}