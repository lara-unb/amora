#ifndef CLASS_FSM_H_
#define CLASS_FSM_H_
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "filters.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class FSM
{
	//states
	enum
	{
		INIT,
		WAIT,
		ACK,
		FOLLOW,
		TOUR_GUIDE,
		SELF_DESTR,
	};
	
	private:
		//typedef void (*fpt)(int)
		typedef struct Buffer
		{
			std_msgs::String str;
			ros::Time timestamp;
		}msg_Buffer;
		
		
		int state;
		bool state_wait;
		std::string speech;
		msg_Buffer buffer;
		Filter* filter;
		
		//void wait();
		void machine();
		void ack();
		void follow();
		void tour_guide();
		void selfdestr();
		void whatstate();
		
	public:
		FSM();
		
		//int getState();
		std::string getState();
		void input(const std_msgs::String::ConstPtr&);
		std_msgs::String output();
		//std_msgs::String output(char *str);
		
		~FSM();
};

#endif //CLASS_FSM_H_