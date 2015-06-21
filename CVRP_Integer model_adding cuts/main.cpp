/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: main.cpp                                                      */
/**********************************************************************/

#include <iostream>
#include <vector>
#include <time.h>
using namespace std;

#include "nodesInfo.h"
#include "vrp.h"


void main(int argc, char **argv)
{
	vector<double> result;	
	{
		nodesInformation input; //the information of nodes
		readNodesFile("A-n44-k7.vrp", input); //read the file
		calCost(input); //

		double cost = 0;
		cost = vrpSolver(input);
		cout<<"the cost is "<<cost<<endl;
		result.push_back(cost);
	}	

}

