/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: nodesInfo.h                                                   */
/*Description: reading the coordinates of all the nodes               */
/**********************************************************************/

#include <vector>
using std::vector; 
#include <string>
using std::string;
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <math.h>
using namespace std;

#include "nodesInfo.h"

/*****************************************************************************/
/* read the information of the nodes information                             */
/*****************************************************************************/
void readNodesFile(char *nodesFile, vector<node>& nodes, int& n)
{
	std::ifstream inFile;
	inFile.open(nodesFile);
	if (!inFile)
	{
		printf("Unable to open file: %s\n", nodesFile);
		exit(-1);
	}

	//read header from datfile
	string line;
	vector<string> lineArr;
	getline(inFile, line);
	splitString(&lineArr, &line, '\t');
	n=atoi(lineArr[0].c_str());
	

	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, '\t');
		double x=atof(classArr[0].c_str()); 
		double y=atof(classArr[1].c_str()); 
		nodes.push_back(node(x,y));		
	}//endwhile

	return;
}

/*****************************************************************************/
/*obtain the cost for all the links                                          */
/*****************************************************************************/
vector<vector<double>> calCost(const vector<node>& nodes, int num)
{
	vector<vector<double>> cost;
	for(int i=0; i<num; i++)
	{
		cost.push_back ( vector<double> (num, 0) );
	}//endfori

	for(int i=0; i<num-1; i++)
	{
		for(int j=i+1; j<num; j++)
		{
			cost.at(i).at(j)=sqrt( pow( (nodes.at(i).first-nodes.at(j).first), 2) + pow( (nodes.at(i).second-nodes.at(j).second), 2) );
			cost.at(j).at(i)=cost.at(i).at(j);
		}//endforj	
	}//endfori

	return cost;

}




/*****************************************************************************/
/*function for parsing the input file                                        */
/*****************************************************************************/
void splitString(std::vector<string> *vec, std::string *line, char delim)
{
	std::istringstream iss(*line);
    string temp;
    while (!iss.eof()) {
        getline(iss, temp, delim);
        if (!temp.empty()) {
            (*vec).push_back(temp);
        }
    }
    return;
}