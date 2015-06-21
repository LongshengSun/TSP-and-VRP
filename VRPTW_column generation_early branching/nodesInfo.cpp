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
void readNodesFile(char *nodesFile, nodesInformation &input)
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

	//read the first line as file name
	getline(inFile, line);
	vector<string> classArr;
	splitString(&classArr, &line, ' ');
	input.name = classArr[0].c_str();


	//read in the basic information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}
		vector<string> classArr;
		splitString(&classArr, &line, ' ');
		if ( classArr[0] == "NUMBER")
			break;		
	}//endwhile

	{
		getline(inFile, line);
		vector<string> classArr;
		splitString(&classArr, &line, ' ');
		input.vehNum= atoi( classArr[0].c_str());
		input.capacity = atof( classArr[1].c_str());
	}

	//read in the coordinates information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		if ( classArr[0] == "CUST" )
		{
			break;
		}//endif		
	}//endwhile

	

	//read in the depot information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		if(classArr.size()>=1)
		{
			node c;
			c.first =  atof( classArr[1].c_str());
			c.second =  atof( classArr[2].c_str());
			input.coordinates.push_back( c );
			input.demand.push_back( atof( classArr[3].c_str()));
			input.rt.push_back( atof( classArr[4].c_str()));
			input.dt.push_back(  atof( classArr[5].c_str()) );
			input.st.push_back(  atof( classArr[6].c_str()) );
		}
	}//endwhile


	input.dimension=input.demand.size();
	return;
}

/*****************************************************************************/
/*obtain the cost for all the links                                          */
/*****************************************************************************/
void calCost(nodesInformation &input)
{
	for(int i=0; i<input.dimension; i++)
	{
		input.cost.push_back ( vector<double> (input.dimension, 0) );
	}//endfori


	for(int i=0; i<input.dimension-1; i++)
	{
		for(int j=i+1; j<input.dimension; j++)
		{
			input.cost.at(i).at(j)=sqrt( pow( (input.coordinates.at(i).first-input.coordinates.at(j).first), 2) 
				+ pow( (input.coordinates.at(i).second-input.coordinates.at(j).second), 2) );
			input.cost.at(j).at(i)=input.cost.at(i).at(j);
		}//endforj	
	}//endfori

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