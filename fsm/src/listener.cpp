#include "ros/ros.h"
#include "std_msgs/String.h"

#include "continuous.h"

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
	//vetor de ponteiros para substituir *argc[]
    char *vetor[5];
    	  vetor[0] = (char*)"./continuous\0";
    	  vetor[1] = (char*)"-lm\0";
    	  vetor[2] = (char*)"/home/aramis/workspaces/hydro/test_ws/src/hello/data/model/language_model.lm\0";
    	  vetor[3] = (char*)"-dict\0";
    	  vetor[4] = (char*)"/home/aramis/workspaces/hydro/test_ws/src/hello/data/model/phonetic_dictionary.dic\0";

    std_msgs::String msg;
    std::string sms;
    //char *argumento = (char*)"-adcdev plughw:1,0 -lm /home/gabriel/AMORA/model_2/5424.lm -dict /home/gabriel/AMORA/model_2/5424.dic";
    std::stringstream ss;
	//Continuous Sphinx
	//continuous(0, (char**)"-adcdev plughw:1,0 -lm /home/gabriel/AMORA/model_2/5424.lm -dict /home/gabriel/AMORA/model_2/5424.dic", sms);
	ps::continuous(5, vetor, sms);//captura de audio; sms: string da captura
	ss << sms;
	
    //ss << "hello world " << count;
    //cout << endl << endl << endl << sms << endl << ss.str() << endl << endl << endl << endl;
    //if(sms.empty())	cout << endl << "EMPTY" << endl << endl << endl << endl;
    if(filters::filter(ss.str()))
    {
    	    if(sms.empty());
    	    else
    	    {
		    msg.data = ss.str();

		    ROS_INFO("%s", msg.data.c_str());

		    chatter_pub.publish(msg);
	    }
    }

    ros::spinOnce();

    //loop_rate.sleep();
  }

  festival_say_text(EST_String("GOOD BYE."));
  return 0;
}
