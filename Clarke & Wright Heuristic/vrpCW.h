/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrpCW.h                                                       */
/*Description: regarding the header file for solving vehicle routing  */
/*             problem using Clark Wright heuristics                  */
/**********************************************************************/

#ifndef VRPCW_H
#define VRPCW_H

#include <vector>
#include "nodesInfo.h"

const double smallNum=0.0001; //define a small number value to assist in comparing
double vrpCW(nodesInformation &input, vector< vector<int> >& routes); //the heuristic method

typedef std::pair<int, int> savingsNodes;
typedef std::pair<double, savingsNodes> sav;
bool cpSavings (sav i,sav j);
void updateRoutes(nodesInformation &input, vector< vector<int> >& routes, vector<sav>& savings);
void mergeRoutes(nodesInformation &input, vector< vector<int> >& routes, vector<sav>& savings, int iFlag, int iFlagR, int jFlag, int jFlagR, int i, int j);

#endif