/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: tspSolver.h                                                   */
/*Description: regarding the header file for solving travelling sales */
/*             mand problem                                            */
/**********************************************************************/

#ifndef TSPSOLVER_H
#define TSPSOLVER_H

#include <ilcplex/ilocplex.h>
#include <vector>


typedef IloArray<IloIntVarArray> IntVarMatrix; //define the int var decision matrix variables type
typedef IloArray<IloNumVarArray> NumVarMatrix; //define the continuous var decision matrix variables type
typedef IloArray<IntVarMatrix> IntVar3D; //define the 3D int decision variables
typedef IloArray<NumVarMatrix> NumVar3D; //define the 3D number decision variables
typedef IloArray<IloExprArray> ExprMatrix; //define the expression 
const double smallNum=0.001; //define a small number value to assist in comparing

//The Miller-Tucker-Zemlin formulation, flag=false for solving integer relaxation
double MTZ(const vector<vector<double>> cost, double & gap, bool flag, int num);
//The Quadratic formulation
double QF(const vector<vector<double>> cost, double &gap, bool flag, int num);
//the multicommodity flow formulation
double MFF(const vector<vector<double>> cost, double &gap, bool flag, int num);
//the shortest path formulation (stage based formulation)
double SPF(const vector<vector<double>> cost, double &gap, bool flag, int num);

#endif