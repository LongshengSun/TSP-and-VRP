/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: main.cpp                                                      */
/**********************************************************************/

#include <iostream>
#include <vector>
#include <time.h>
using namespace std;

#include "nodesInfo.h"
#include "vrpCW.h"


void main(int argc, char **argv)
{
    nodesInformation input; //the information of nodes
	readNodesFile("A-n45-k7.vrp", input); //read the file
	calCost(input); //

	double cost = 0;
	vector< vector<int> > routes;
	cost = vrpCW(input, routes);
	cout<<"the heuristic cost is"<<cost;
}

