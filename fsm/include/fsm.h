//Header fsm
#ifndef FSM_H
#define FSM_H
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "festival.h"
#include <iostream>
#include <string>
#include <ctime>

using namespace std;

namespace fsm
{
	enum command
	{
		follow_me,
		show_the_lara,
		tour_guide_mode = 1,
	};

	void hi();

	void yes();

	void follow();
	
	void init();

	void say(const char* sms);
	
	void say(string sms);
	
	void say(int n);
	
	void wait();
	
	void ack(ros::Time ts, int* state, bool *state_wait);
	
	void tour_guide();
	
	void check(const char *str, int *state, bool *state_wait);
	
	void iam_check(string str);
	//void ack_chatterCallback(const std_msgs::String::ConstPtr& msg);
}

#endif //FSM_H
