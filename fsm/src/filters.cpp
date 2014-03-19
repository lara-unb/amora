#include "filters.h"

void filters::openfile(vector<string> *vecmap)
{
	static int n = 0;//variavel estatica local
	if(n != 0)	return;//Sair caso a funcao ja tenha sido chamada, evitar acesso ao HD
	
	ifstream filemap("/home/aramis/workspaces/hydro/test_ws/src/hello/data/model/corpus.txt");//Abrir arquivo texto com os comandos
	string linha;
	
	if(!filemap.is_open())//Problema na abertura do arquivo
	{
		cout << "Unable to open the file corpus.txt" << endl;
		return;
	}
	
	while(filemap.good())//
	{
		getline(filemap, linha);
		locale loc;
		for(string::size_type i = 0; i < linha.length(); i++)
		{
			linha[i] = toupper(linha[i],loc);
		}
		vecmap->push_back(linha);//add a linha como ultimo elemento de vecmap 
	}
	
	filemap.close();
	n++;
}

string filters::search_command(string command)
{
	static vector<string> vetormap;
	
	openfile(&vetormap);
	
	static map<string,string> mapa;
	static int n = 0;
	map<string,string>::iterator it;
	
	if(n == 0)
	{
		for(unsigned int i = 0; i < vetormap.size(); i++)
		{
			mapa[vetormap.at(i)] = "found";
		}
	}
	
	it = mapa.find(command);
//	cout << endl << endl << endl << command << endl << endl << endl;
	if(it != mapa.end())
	{
		return string("found");
	}
	else return string("NOT FOUND");
	
	n++;
}

bool filters::filter(string command)
{
	if(search_command(command) == "found")	return true;
	else	return false;
}

