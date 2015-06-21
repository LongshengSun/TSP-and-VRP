/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrptw.h                                                       */
/*Description: regarding the header file for solving vehicle routing  */
/*             problem with time windows                              */
/**********************************************************************/

#ifndef VRPTW_H
#define VRPTW_H

#include <ilcplex/ilocplex.h>
#include <vector>
#include "nodesInfo.h"


typedef IloArray<IloIntVarArray> IntVarMatrix; //define the int var decision matrix variables type
typedef IloArray<IloNumVarArray> NumVarMatrix; //define the continuous var decision matrix variables type
typedef IloArray<IntVarMatrix> IntVar3D; //define the 3D int decision variables
typedef IloArray<NumVarMatrix> NumVar3D; //define the 3D number decision variables
typedef IloArray<IloExprArray> ExprMatrix; //define the expression 
const double smallNum=0.0001; //define a small number value to assist in comparing

double columnGenerationVRPTW(nodesInformation &input, vector< vector<int> >& pathSet, vector<double>& costPathSet, vector< vector<int> >& aPathSet);

const double M=1350;

#endif