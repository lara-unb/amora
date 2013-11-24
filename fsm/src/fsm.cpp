#include "fsm.h"

void fsm::hi()
{
	fsm::say("GOOD MORNING. MY NAME IS ARAMIS AND I WILL BE YOUR TOUR GUIDE.");
	//festival_say_text(EST_String("Good Morning. My name is Aramis and I will be your tour guide."));
}

void fsm::yes()
{
	fsm::say("YES MASTER.");
}

void fsm::follow()
{
	fsm::say("I WILL FOLLOW YOU NOW.");
}

void fsm::init()
{
	int heap_size = FESTIVAL_HEAP_SIZE;  // default scheme heap size
	int load_init_files = 1; // we want the festival init files loaded

	festival_initialize(load_init_files,heap_size);
	 
	fsm::hi();
}

void fsm::say(const char* sms)
{
	cout << sms << endl;
	festival_say_text(EST_String(sms));
}

void fsm::say(string sms)
{
	cout << sms << endl;
	festival_say_text(EST_String(sms.c_str()));
}

void fsm::say(int n)
{
	switch(n)
	{
	   	case '0':
	   		fsm::say("I WILL FOLLOW YOU NOW");
	   		break;
	   	case '1':
	   		fsm::say("");
	   		break;
	   	
	   	case '2':
	   		fsm::say("I WILL BE YOUR TOUR GUIDE");
	   		break;
	}
}

void fsm::wait()
{
	return;
}

void fsm::ack(ros::Time ts, int *state, bool *state_wait)
{
	static bool primeiro = true;

	if(primeiro == true)
	{
		fsm::say("YES MASTER");
	}
	
	ros::Time now = ros::Time::now();
	ros::Duration time;
	
	time = now - ts;
	ros::Duration ten_seconds(10.0);
	//cout << time << endl;
	primeiro = false;
	if(time > ten_seconds)
	{
		*state = 1;
		*state_wait = false;
		primeiro = true;
	}
}

void fsm::tour_guide()
{
	fsm::say("I WILL BE YOUR TOUR GUIDE.");
}


void fsm::check(const char *str, int *state, bool *state_wait)
{
	if(strcmp(str, "HI ARAMIS") == 0)
	{
		*state_wait = true;
		*state = 2;
		return;
	}
	if(*state_wait == true)
	{
		if(strcmp(str, "SHOW ME THE LARA") == 0)	*state = 3 + show_the_lara;
		else if(strcmp(str, "TOUR GUIDE MODE") == 0)	*state = 3 + tour_guide_mode;
		else if(strcmp(str, "FOLLOW ME") == 0)	*state = 3 + follow_me;
		else	fsm::iam_check(string(str));
	}
}

void fsm::iam_check(string str)
{
	size_t n = str.find("ARAMIS"), i = str.find("I AM ");
	
	//cout << name << endl;
	if(n != string::npos)
	{
		string name = str.substr(n);
		if(strcmp(name.c_str(),"ARAMIS") == 0)
		{
			fsm::say("NO");
			fsm::say("I AM ARAMIS.");
		}
	}
	else if(i != string::npos)
	{
		string hi("HI "), fala = hi + str.substr(5);
		fsm::say(fala);
	}
}
