/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrpCW.cpp                                                */
/*Description: regarding the implemention for solving VRP            */
/*             problem using Clark and Wright heuristics             */
/**********************************************************************/
#include <vector>
using std::vector;
#include <iostream>
#include <functional>
using namespace std;
#include <algorithm>    // std::sort

#include "vrpCW.h"
#include "nodesInfo.h"

//the Clark and Wright heuristics
double vrpCW(nodesInformation& input, vector< vector<int> >& routes)
{

	int num = input.dimension;
	int d = input.depot-1;

	vector<sav> savings;
	vector<vector<int>> currentRoutes;

	//update the savings for using n trucks
	for(int i=1; i<num-1; i++)
	{
		for(int j=i+1; j<num; j++)
		{
			sav savingsUnit;
			savingsUnit.first= input.cost.at(d).at(i) + input.cost.at(d).at(j) - input.cost.at(i).at(j);
			savingsUnit.second.first = i;
			savingsUnit.second.second = j;
			savings.push_back(savingsUnit);
		}//endforj
	}//endfori

	//sort the savings
	std::sort(savings.begin(), savings.end(), cpSavings);

	//update the routes
	while(savings.size()>0)
	{
		updateRoutes(input, routes, savings);
	}
	

	double cost=0;
	for(int r=0; r<routes.size(); r++)
	{
		if(routes.at(r).size()>0)
		{
			cost=cost+input.cost.at(d).at(routes.at(r).at(0));
			cost=cost+input.cost.at(d).at(routes.at(r).at( routes.at(r).size()-1  )  );
			if(routes.at(r).size()>=2){
				for(int c=0; c<routes.at(r).size()-1-1; c++)
				{
					int p=routes.at(r).at(c);
					int q=routes.at(r).at(c+1);
					cost = cost + input.cost.at(p).at(q);
				}//endfor
			}
		}//endif		
	}//endforr

	return cost;
}

//used for the comparison of savings
bool cpSavings (sav i,sav j)
{ 
	return (i.first>j.first); 
}

void updateRoutes(nodesInformation &input, vector< vector<int> >& routes, vector<sav>& savings)
{
	int i = savings.at(0).second.first;
	int j = savings.at(0).second.second;

	if(routes.size()>0)
	{
		int iFlag = -2; int jFlag = -2; int iFlagR = -2; int jFlagR = -2; //meaning not found
		for(int r=0; r<routes.size(); r++)
		{
			std::vector<int>::iterator itI;
			std::vector<int>::iterator itJ;
			itI = std::find(routes.at(r).begin(), routes.at(r).end(), i);
			itJ = std::find(routes.at(r).begin(), routes.at(r).end(), j);

			//considering the i node
			if(iFlag==-2 && itI != routes.at(r).end() ) //if it was not found now we find it
			{
				if(itI==routes.at(r).begin())//found in the beginning
				{
					iFlag = 0;
					iFlagR = r;
				}
				else if (itI==(routes.at(r).end()-1)) //found in the end
				{
					iFlag = routes.at(r).size()-1; 
					iFlagR = r;
				}
				else
				{
					iFlag = -1; //found in the interior
					iFlagR = r;
				}
			}//endifi

			//considering the j node
			if(jFlag==-2 && itJ != routes.at(r).end() ) //if it was not found now we find it
			{
				if(itJ==routes.at(r).begin())//found in the beginning
				{
					jFlag = 0;
					jFlagR = r;
				}
				else if (itJ==(routes.at(r).end()-1)) //found in the end
				{
					jFlag = routes.at(r).size()-1; 
					jFlagR = r;
				}
				else
				{
					jFlag = -1; //found in the interior
					jFlagR = r;
				}
			}//endifi
		}//endfor

		//now merge the routes
		mergeRoutes(input, routes, savings, iFlag, iFlagR, jFlag, jFlagR, i, j);

		savings.erase(savings.begin());//erase the element that has been processed

	}//endif
	else
	{
		//construct the first route
		vector<int> routeUnit;
		routeUnit.push_back(i);
		routeUnit.push_back(j);
		if(input.demand.at(i) + input.demand.at(j) < input.capacity)
		{
			routes.push_back(routeUnit);
		}
		savings.erase(savings.begin());//erase the element that has been processed
	}//endelse

}

void mergeRoutes(nodesInformation &input, vector< vector<int> >& routes, vector<sav>& savings, int iFlag, int iFlagR, int jFlag, int jFlagR, int i, int j)
{
	if(iFlag == -2 && jFlag == -2) //both not found, construct new routes
	{
		vector<int> routeUnit;
		routeUnit.push_back(i);
		routeUnit.push_back(j);
		if(input.demand.at(i) + input.demand.at(j) < input.capacity)
		{
			routes.push_back(routeUnit);
		}//endif
	}
	else if (iFlag == -2 && jFlag >= 0 ) //i not found but j is found and not interior
	{
		double d=0;
		for(int m=0; m<routes.at(jFlagR).size(); m++)
		{
			int c = routes.at(jFlagR).at(m);
			d=d+input.demand.at( m );
		}
		if(d<input.capacity)
			routes.at(jFlagR).insert( routes.at(jFlagR).begin()+jFlag, i);
	}
	else if (jFlag == -2 && iFlag >= 0) //j not found but i is found and not interior
	{
		double d=0;
		for(int m=0; m<routes.at(iFlagR).size(); m++)
		{
			int c = routes.at(iFlagR).at(m);
			d=d+input.demand.at( m );
		}
		if(d<input.capacity)
			routes.at(iFlagR).insert( routes.at(iFlagR).begin()+iFlag, j);
	}
	else if(jFlag>=0 && iFlag>=0 && iFlagR != jFlagR) //if both of them are found and not interior points
	{
		//obtain the demand
		double d=0;
		for(int m=0; m<routes.at(jFlagR).size(); m++)
		{
			int c = routes.at(jFlagR).at(m);
			d=d+input.demand.at( m );
		}
		for(int m=0; m<routes.at(iFlagR).size(); m++)
		{
			int c = routes.at(iFlagR).at(m);
			d=d+input.demand.at( m );
		}

		if(d<input.capacity)
		{
			vector<int> routeUnit;
			if((iFlag ==0 && jFlag ==0) || (iFlag >0 && jFlag >0) )//both found in the front
			{
				//construct the new route
				std::reverse(routes.at(iFlagR).begin(), routes.at(iFlagR).end());
				routeUnit.reserve(routes.at(iFlagR).size()+routes.at(jFlagR).size());
				routeUnit.insert(routeUnit.end(), routes.at(iFlagR).begin(), routes.at(iFlagR).end());
				routeUnit.insert(routeUnit.end(), routes.at(jFlagR).begin(), routes.at(jFlagR).end());
				routes.push_back(routeUnit);
				routes.at(iFlagR).clear();
				routes.at(jFlagR).clear();
			}	
			else
			{
				routeUnit.reserve(routes.at(iFlagR).size()+routes.at(jFlagR).size());
				routeUnit.insert(routeUnit.end(), routes.at(iFlagR).begin(), routes.at(iFlagR).end());
				routeUnit.insert(routeUnit.end(), routes.at(jFlagR).begin(), routes.at(jFlagR).end());
				routes.push_back(routeUnit);
				routes.at(iFlagR).clear();
				routes.at(jFlagR).clear();
			}
		}//endifd
	}//endelseif
}