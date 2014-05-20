#include "filters.h"

Filter* Filter::p = NULL;

Filter::Filter()
{
	//this->setPath(std::string("/home/aramis/workspaces/hydro/catkin_ws/src/fsm/data/model/answer.txt"));
}

Filter* Filter::getInstance()
{
	if(!p)
	{
		p = new Filter();
		p->setPath(std::string("/home/gastd/workspaces/hydro/catkin_ws/src/fsm/data/model/answer.txt"));
		if(!p)	p->setPath(std::string("/home/gastd/workspaces/hydro/catkin_ws/src/fsm/data/model/answer.txt"));
		//p->openfile();
	}
	
	return p;
}

void Filter::setPath(char* new_path)
{
	this->path << new_path;
	p->openfile();
}

void Filter::setPath(std::string new_path)
{
	this->path << new_path;
	p->openfile();
}

void Filter::openfile()
{
	std::ifstream filemap(path.str().c_str());
	std::string line, command, answer;
	
	if(!filemap.is_open())
	{
		std::cout << "Unable to open file ...txt" << std::endl;
		return;
	}
	
	while(filemap.good())
	{
		getline(filemap, line);
		std::locale loc;
		for(std::string::size_type i = 0; i < line.length(); i++)
		{
			line[i] = toupper(line[i], loc);
		}
		
		unsigned pos = line.find('*');
		command = line.substr(0,pos);
		answer = line.substr(pos+1);

		this->veccommand.push_back(command);
		this->vecanswer.push_back(answer);
	}
	
	this->fillmap();
	
	filemap.close();
}

void Filter::fillmap()
{
	for(unsigned int i = 0; i < veccommand.size(); i++)
	{
		Map[veccommand.at(i)] = vecanswer.at(i);
	}
}

std::string Filter::answer(std::string command)
{
	std::map<std::string, std::string>::iterator it;
	it = Map.find(command);
	if(it != Map.end())
	{
		return Map.at(command);
	}
	else return std::string("WHAAAT!?");
}

void Filter::release()
{
	delete p;
}

Filter::~Filter()
{
//
}
