/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: main.cpp                                                      */
/**********************************************************************/

#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>
using namespace std;

#include "nodesInfo.h"
#include "tsp_lagrangian.h"

void main(int argc, char **argv)
{
	vector<double> boundsLagrangian;
	vector<double> time;
	for(int n=15; n<=30; n=n+5)
	{
		for(int s=1; s<=5; s++)
		{
			//read the data file and calculate the link cost
			char fileName[50];
			sprintf_s(fileName, "TSP_instance_n_%d_s_%d.dat",n,s);
			vector<node> nodes; //the coordinates of nodes
			int num; //the number of nodes
			readNodesFile(fileName, nodes, num); //read the file
			vector<vector<double>> cost; //the link cost
			cost=calCost(nodes, num);//calculate the cost
			
			clock_t start, end;//record the time
			start=clock();
			double boundLag=tsp_lagrangian(cost, num, 0.5, 100, 0.0001, 0.05);
			end =clock();
			double t=(double) (end-start)/CLOCKS_PER_SEC;

			//obtain the results
			boundsLagrangian.push_back(boundLag);
			time.push_back(t);
		}//endfors
	}//endforn

	//write the results to file
	char outFileName[50]="result.csv";
	ofstream resultFile;
	resultFile.open(outFileName);
	for(int i=0; i<time.size(); i++)
	{
		resultFile<<boundsLagrangian.at(i)<<","<<time.at(i)<<endl;
	}
	resultFile.close();
}

