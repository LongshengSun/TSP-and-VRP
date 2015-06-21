/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: tsp_lagrangian.h                                              */
/*Description: lagrangian relaxation to obtain an upper bound         */
/**********************************************************************/

#ifndef TSPLAGRANGIAN_H
#define TSPLAGRANGIAN_H

#include <vector>
#include <queue> // std::priority_queue
#include <set>

struct item {
	int u;
	int pred;
	double cost;
};

class itemComparison
{
  bool reverse;
public:
  itemComparison(const bool& revparam=true)
    {reverse=revparam;}
  bool operator() (const item& lhs, const item&rhs) const
  {
	  if (reverse) return (lhs.cost>rhs.cost);
	  else return (lhs.cost<rhs.cost);
  }
};

typedef std::priority_queue<item,std::vector<item>,itemComparison> mst_pq_type;
double _1tree(vector<vector<double>> cost, int num, vector<double> uMultipliers, vector<vector<int>>& xe);
double mst(vector<vector<double>> cost, int num, vector<vector<int>>& xe);
double tsp_lagrangian(vector<vector<double>> cost, int num, double M, double K, double epsilon, double rho);
const double smallNum=0.001; //define a small number value to assist in comparing
const double bigNum=1000000; 

#endif