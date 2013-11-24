//Header Filters
#ifndef FILTERS_H
#define FILTERS_H
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <locale>
#include <cstring>


using namespace std;

namespace filters
{
	void openfile(vector<string> *vecmap);

	string search_command(string command);

	bool filter(string command);
}

#endif //FILTERS_H
