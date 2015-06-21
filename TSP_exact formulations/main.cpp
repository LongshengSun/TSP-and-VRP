/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: main.cpp                                                      */
/**********************************************************************/

#include <iostream>
#include <vector>
#include <time.h>
using namespace std;

#include "nodesInfo.h"
#include "tspSolver.h"

struct result
{
	double lrv; //the linear relaxatio value
	double biv; //the best integer value
	double og; //the optimality gap
	double time; //time for solving the cases

};

void printResults(const vector<vector<result>>& results)
{
	char outFileName[50]="result.csv";
	ofstream resultFile;
	resultFile.open(outFileName);
	int n=results.at(0).size();
	for(int i=0; i<n; i++)
	{
		for(int f=0; f<4; f++)
		{
			resultFile<<results.at(f).at(i).lrv<<",";
		}
		for(int f=0; f<4; f++)
		{
			resultFile<<results.at(f).at(i).biv<<",";
		}
		for(int f=0; f<4; f++)
		{
			resultFile<<results.at(f).at(i).og<<",";
		}
		for(int f=0; f<4; f++)
		{
			resultFile<<results.at(f).at(i).time<<",";
		}
		resultFile<<endl;
	}
}

void main(int argc, char **argv)
{
	vector<vector<result>> results;
	for(int f=1; f<=4; f++)
	{
		vector<result> rf; //result for formulation f
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
				cost=calCost(nodes, num); //calculate the cost

				double t=-1, bestIntValue=-1, gap=-1, lr=-1;
				result r;
				clock_t start, end;
				switch (f)
				{
				case 1:	
					start=clock();	
					bestIntValue=MTZ(cost, gap, true, num);
					lr=MTZ(cost, gap, false, num);
					end=clock();
					t=(double) (end-start)/CLOCKS_PER_SEC;
					r.lrv=lr;
					r.biv=bestIntValue;
					r.og=gap;
					r.time=t;
					rf.push_back(r);					
					break;
				case 2:
					start=clock();	
					bestIntValue=MFF(cost, gap, true, num);
					lr=MFF(cost, gap, false, num);
					end=clock();
					t=(double) (end-start)/CLOCKS_PER_SEC;
					r.lrv=lr;
					r.biv=bestIntValue;
					r.og=gap;
					r.time=t;
					rf.push_back(r);					
					break;
				case 3:
					start=clock();	
					bestIntValue=SPF(cost, gap, true, num);
					lr=SPF(cost, gap, false, num);
					end=clock();
					t=(double) (end-start)/CLOCKS_PER_SEC;
					r.lrv=lr;
					r.biv=bestIntValue;
					r.og=gap;
					r.time=t;
					rf.push_back(r);					
					break;
				case 4:
					start=clock();	
					bestIntValue=QF(cost, gap, true, num);
					lr=QF(cost, gap, false, num);
					end=clock();
					t=(double) (end-start)/CLOCKS_PER_SEC;
					r.lrv=lr;
					r.biv=bestIntValue;
					r.og=gap;
					r.time=t;
					rf.push_back(r);					
					break;
				default:
					cout<<"wrong input value"<<endl;
					break;
				}//endswitch
			}//endfors
		}//endforn
		results.push_back(rf);
	}//endforf
	
	//print out the results
	printResults(results);

    /*vector<node> nodes; //the coordinates of nodes
	int num; //the number of nodes
	readNodesFile("TSP_instance_n_10_s_676.dat", nodes, num); //read the file
	vector<vector<double>> cost; //the link cost
	cost=calCost(nodes, num);

	double gap=-1;
	
	double bestIntValue=MTZ(cost, gap, true, num);
	double lr=MTZ(cost, gap, false, num);*/
}

