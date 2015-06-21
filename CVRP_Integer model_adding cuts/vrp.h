/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrp.h                                                         */
/*Description: regarding the header file for solving vehicle routing  */
/*             problem                                                */
/**********************************************************************/

#ifndef VRPSOLVER_H
#define VRPSOLVER_H

#include <ilcplex/ilocplex.h>
#include <vector>
#include "nodesInfo.h"


typedef IloArray<IloIntVarArray> IntVarMatrix; //define the int var decision matrix variables type
typedef IloArray<IloNumVarArray> NumVarMatrix; //define the continuous var decision matrix variables type
typedef IloArray<IntVarMatrix> IntVar3D; //define the 3D int decision variables
typedef IloArray<NumVarMatrix> NumVar3D; //define the 3D number decision variables
typedef IloArray<IloExprArray> ExprMatrix; //define the expression 
const double smallNum=0.0001; //define a small number value to assist in comparing


double vrpSolver(nodesInformation &input);
vector< vector<int> > tours(vector< vector< vector<int> > > vars, nodesInformation &input);
void makeCuts(const vector<vector<vector<int>>>& vars, IntVar3D Dvars, IloExprArray lhs, IloNumArray rhs, IloEnv vrpEnv, nodesInformation &input);
bool inTour(const vector<int>& t, int n);
bool addCuts(IntVar3D vars, IloNum eps, nodesInformation & input, IloModel model, IloCplex cplex);
const int K = 7;

#endif