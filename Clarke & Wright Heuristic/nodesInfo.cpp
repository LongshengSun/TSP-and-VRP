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

	//read in the basic information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		if( classArr[0] == "NAME")
		{
			input.name = classArr[2].c_str();
		}
		else if ( classArr[0] == "DIMENSION" )
		{
			input.dimension = atoi( classArr[2].c_str() );
		}
		else if ( classArr[0] == "CAPACITY" )
		{
			input.capacity = atof( classArr[2].c_str());
		}
		else if ( classArr[0] == "NODE_COORD_SECTION" )
		{
			break;
		}	
	}//endwhile

	//read in the coordinates information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		if ( classArr[0] == "DEMAND_SECTION" )
		{
			break;
		}//endif

		node c;
		//read in the node coordinate
		c.first = atof( classArr[1].c_str());
		c.second = atof( classArr[2].c_str());
		input.coordinates.push_back(c);
	}//endwhile

	//read in the demand information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		if ( classArr[0] == "DEPOT_SECTION" )
		{
			break;
		}//endif	

		//read in the node demand
		input.demand.push_back( atof( classArr[1].c_str()));
		
	}//endwhile

	//read in the depot information
	while(!inFile.eof())
	{
		getline(inFile, line);
		if (line.empty()){continue;}

		vector<string> classArr;
		splitString(&classArr, &line, ' ');

		//read in the deposit
		input.depot = atoi( classArr[0].c_str());
		break;
	}//endwhile

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