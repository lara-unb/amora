/**
 * @file     ar_map.h
 * @author   George Andrew Brindeiro and Mateus Mendelson
 * @date     06/10/2012
 *
 * @brief Data structure to contain a map of ARMarkers for the artoolkit_localization package
 *
 * This class contains a map of markers with coordinates, accessible by their name as specified in ar_pose
 *
 * Contact: georgebrindeiro@lara.unb.br
 *
 * Revisions:
 * [06/10/2012] Created
 */

#ifndef AR_MAP_H
#define AR_MAP_H

// Standard C++ libraries
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// Additional
#include <algorithm>
#include <stdexcept>
#include <iterator>
#include <sstream>
#include <cstdlib>

// Choose whether to map or not
#define MAPPING true

using namespace std;

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

class ARMap
{
    public:
    	#ifdef MAPPING
    	ARMap(std::string map_path = "cfg/outputMap.map")
        {
            map_path_ = map_path;

            Init();
        }
    	#else
        ARMap(std::string map_path = "cfg/lara.map")
        {
            map_path_ = map_path;

            Init();
        }
        #endif

        ~ARMap() {};

        void Init();

		// Reading Functions

		bool read(string filename);

		bool parseLine(string line);

		// Map functions

		bool empty();

		int getSize();

		// Vector functions

		vector<double>& get(string key);

		vector<double>& get(int key);

		string& name(string key);

		string& name(int key);

		// Misc

		void print();

		map< string, vector <double> >::iterator& begin();

	 	map< string, vector <double> >::iterator& end();

    private:
        std::string map_path_;		// File with list of ARMarkers

		map< string, vector <double> > ar_map;
		map< string, string > ar_name;
		map< string, vector <double> >::iterator begin_iterator;
		map< string, vector <double> >::iterator end_iterator;

        DISALLOW_COPY_AND_ASSIGN(ARMap);
};

#endif //AR_MAP_H
