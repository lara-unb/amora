/**
 * @file    ar_map.cpp
 * @author  George Andrew Brindeiro and Mateus Mendelson
 * @date    06/10/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#include <artoolkit_localization/ar_map.h>

#include <ros/ros.h>

double StrToDouble(std::string const& s)
{
    std::istringstream iss(s);
    double value;
    if (!(iss >> value)) throw std::runtime_error("invalid double");
    return value;
}

std::string IntToStr(int number)
{
	stringstream ss;//create a stringstream
	ss << number;//add number to the stream
	return ss.str();//return a string with the contents of the stream
}

void ARMap::Init()
{
	#if MAPPING
		// mapeando
	#else
		// Open map file
		if(!read(map_path_))
			ROS_ERROR("Could not open file %s", map_path_.c_str());
	#endif
}

bool ARMap::read(string filename)
{
	ifstream map_file(filename.c_str(), ifstream::in);

	if(map_file.fail())
		return false;

	string line;

	while(map_file.good())
	{
		getline(map_file, line);
		parseLine(line);
	}

	map_file.close();

	return true;
}

bool ARMap::parseLine(string line)
{
	if((line[0] == '#') || (line.size() == 0))
		return false;

	// construct a stream from the string
	stringstream ss(line);

	// use stream iterators to copy the stream to the vector as whitespace separated strings
	istream_iterator<string> it(ss);
	istream_iterator<string> end;
	vector<string> tokens(it, end);

	ROS_DEBUG_STREAM("tokens size: " << tokens.size());

	string key(IntToStr(ar_map.size()));

	ar_name[key] = tokens[0];
	tokens.erase(tokens.begin());

	transform(tokens.begin(), tokens.end(), std::back_inserter(ar_map[key]), StrToDouble);////////////////////////////////////////////////////////////

	ROS_DEBUG_STREAM("ar_map[ " << key << "] size: " << ar_map[key].size());

	return true;
}

bool ARMap::empty()
{
	return ar_map.empty();
}

int ARMap::getSize()
{
	return ar_map.size();
}

vector<double>& ARMap::get(string key)
{
	return ar_map[key];
}

vector<double>& ARMap::get(int key)
{
	return get(IntToStr(key));
}

string& ARMap::name(string key)
{
	return ar_name[key];
}

string& ARMap::name(int key)
{
	return name(IntToStr(key));
}

void ARMap::print()
{
	cout << "[" << map_path_ << "]" << endl;

	if(empty())
	{
		cout << "ARMap is empty!" << endl;
	}
	else
	{
		cout << "id x y" << endl;

		for(map< string, vector <double> >::iterator im = ar_map.begin(); im != ar_map.end(); im++)
		{
			cout << im->first << ":";

			for(vector<double>::iterator iv = im->second.begin(); iv != im->second.end(); iv++)
				cout << " " << *iv;

			cout << endl;
		}
	}

	cout << endl;
}

map< string, vector <double> >::iterator& ARMap::begin()
{
	begin_iterator = ar_map.begin();
	return begin_iterator;
}

map< string, vector <double> >::iterator& ARMap::end()
{
	end_iterator = ar_map.end();
	return end_iterator;
}
