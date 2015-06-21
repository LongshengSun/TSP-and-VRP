/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: tsp_lagrangian.cpp                                            */
/*Description: lagrangian relaxation to obtain an upper bound         */
/**********************************************************************/


#include <vector>
using std::vector;
#include <iostream>
using namespace std;
#include <algorithm> //min_element
#include <numeric>  // std::accumulate
#include <limits>
#include <math.h>       /* pow */

#include "tsp_lagrangian.h"

/*****************************************************************************/
/* the lagrangian relaxation using subgradient search to find lower bound    */
/*****************************************************************************/
double tsp_lagrangian(vector<vector<double>> cost, int num, double M, double K, double epsilon, double rho)
{
	int t=0;
	//initilize Lagrangian multipliers
	vector<double> uMultipliers(num, 0);
	double Lt=0;
	double Lt1=std::numeric_limits<double>::min();	
	bool opFlag=true;
	double bestBound=0;

	do
	{
		opFlag=true;
		Lt=Lt1;
		vector<vector<int>> xe;
		for(int i=0; i<num; i++)
		{
			xe.push_back ( vector<int> (num, 0) );
		}//endfori
		Lt1=_1tree(cost, num, uMultipliers, xe);
		//update the bound
		if(Lt1>bestBound)
		{
			bestBound=Lt1;
		}//endif

		vector<double> direction;
		//obtain the direction
		for(int i=0; i<num; i++)
		{
			double xSum=0;
			for(int j=0; j<num; j++)
			{			
				if(xe.at(i).at(j)==1)
				{
					xSum=xSum+1;
				}//endif
			}//endforj
			xSum=2-xSum;
			if(abs(xSum)>smallNum)
			{
				opFlag=false;
			}//endif
			direction.push_back(xSum);
		}//endfori

		//update the multipliers
		for(int i=0; i<num; i++)
		{
			double step=M*pow(rho,t);
			uMultipliers.at(i)=step*direction.at(i)+uMultipliers.at(i);
		}//endfori

		t=t+1;
	}
	while(t<=K && abs(Lt1-Lt)>epsilon && !opFlag);

	return Lt1;

}


/*****************************************************************************/
/* 1tree                                                                     */
/*****************************************************************************/
double _1tree(vector<vector<double>> cost, int num, vector<double> uMultipliers, vector<vector<int>>& xe)
{
	for(int i=0; i<num; i++)
	{
		for(int j=0; j<num; j++)
		{
			if(i!=j)
				cost.at(i).at(j)=cost.at(i).at(j)-uMultipliers.at(i)-uMultipliers.at(j);
		}//endforj
	}//endfori

	//finding the minimum spanning tree with updated cost
	double Lt=mst(cost, num, xe);

	vector<double> iniVec(cost.at(0).begin()+1, cost.at(0).end());
	//sort the vector
	int a1=min_element(iniVec.begin(), iniVec.end())-iniVec.begin();
	Lt=Lt+iniVec.at(a1);
	iniVec.at(a1)=bigNum;
	int a2=min_element(iniVec.begin(), iniVec.end())-iniVec.begin();
	Lt=Lt+iniVec.at(a2);

	xe.at(0).at(a1+1)=1;
	xe.at(0).at(a2+1)=1;

	double uSum=accumulate(uMultipliers.begin(), uMultipliers.end(), 0.0);

	Lt=Lt+2*uSum;//adding the u summation value

	return Lt;
	
}

/*****************************************************************************/
/* the minimum spanning tree (Prim's)                                        */
/*****************************************************************************/
double mst(vector<vector<double>> cost, int num,  vector<vector<int>>& xe)
{
	double Lt=0; 
	vector<bool> nodeDone(num, false);
	vector<double> key(num, bigNum);
	 mst_pq_type mst_pq;
	 for(int n=2; n<num; n++)
	 {
		 item t;
		 t.u=n, t.pred=-1, t.cost=bigNum;
		 mst_pq.push(t);
	 }//endforn
	 item t1;
	 t1.u=1, t1.pred=-1, t1.cost=0;
	 mst_pq.push(t1);
	 key.at(1)=0;

	 int n=1;
	 while(true)
	 {
		 int uNode=mst_pq.top().u;
		 mst_pq.pop();//extract the minimum
		if(!nodeDone.at(uNode))
		{
			 //cout<<uNode<<",";
			 n=n+1;
			 for(int v=1; v<num; v++)
			 {
				 if((!nodeDone.at(v))&&cost.at(uNode).at(v)<=key.at(v)&&v!=uNode)
				 {
					 item temp;
					 temp.pred=uNode, temp.u=v, temp.cost=cost.at(uNode).at(v);
					 key.at(v)=cost.at(uNode).at(v);
					 mst_pq.push(temp);
				 }//endif
			 }//endforv
			 xe.at( mst_pq.top().pred).at(mst_pq.top().u )=1;
			 Lt=Lt+cost.at( mst_pq.top().pred).at(mst_pq.top().u );
		}
		
		 nodeDone.at(uNode)=true;
		 if(n==num)
		 {
			break;
		 }
	 }//endwhile

	 return Lt;
}