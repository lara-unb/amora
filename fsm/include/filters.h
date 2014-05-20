#ifndef FILTERS_H_
#define FILTERS_H_
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <locale>
#include <cstring>

class Filter
{
	private:
		static Filter* p;
		std::string command;
		std::stringstream path;
		std::vector<std::string> veccommand;
		std::vector<std::string> vecanswer;
		std::map<std::string, std::string> Map;
		
		Filter();
		void openfile();
		void fillmap();
	
	public:
		static Filter* getInstance();

		void setPath(char *);
		void setPath(std::string);
		std::string answer(std::string);
		void release();
		
		~Filter();
};

//extern Filter filter;

#endif //FILTERS_H_
