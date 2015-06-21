/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: main.cpp                                                      */
/**********************************************************************/

#include <iostream>
#include <vector>
#include <time.h>
using namespace std;

#include "nodesInfo.h"
#include "vrptw.h"

void initPathSet(const nodesInformation &input, vector< vector<int> >& pathSet, vector<double>& costPathSet, vector< vector<int> >& aPathSet);

void main(int argc, char **argv)
{
    nodesInformation input; //the information of nodes
	readNodesFile("RC101.txt", input); //read the file
	calCost(input); //

	vector< vector<int> > pathSet; //the set of paths
	vector<double> costPathSet; //the cost for the paths
	vector< vector<int> > aPathSet; //the node path realtion

	initPathSet(input, pathSet, costPathSet, aPathSet);


	columnGenerationVRPTW(input, pathSet, costPathSet, aPathSet);


}

void initPathSet(const nodesInformation &input, vector< vector<int> >& pathSet, vector<double>& costPathSet, vector< vector<int> >& aPathSet)
{
	int num = input.dimension;
	for(int i=1; i<num; i++)
	{
		vector<int> p;
		p.push_back(i);
		pathSet.push_back(p);
		costPathSet.push_back( input.cost.at(0).at(i)*2 );
		vector<int> ap(input.vehNum, 0);
		ap.at(i-1)=1;
		aPathSet.push_back(ap);
	}

	//obtain the initial cost
	double cost =0;
	for(int i=0; i<num-1; i++)
	{
		cost = cost + costPathSet.at(i);
	}
	cout<<"initial cost is "<<cost<<endl;
}

