/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrptw.cpp                                                   */
/*Description: regarding the implemention for solving VRP            */
/*             problem with time windows                              */
/**********************************************************************/
#include <ilcplex/ilocplex.h>
#include <vector>
using std::vector;
#include <iostream>
#include <functional>
using namespace std;

#include "vrptw.h"
#include "nodesInfo.h"

double columnGenerationVRPTW(nodesInformation &input, vector< vector<int> >& pathSet, vector<double>& costPathSet, vector< vector<int> >& aPathSet)
{
    IloEnv env;

   try {
	   /********************************/
	   /*regarding the master problem  */
	   /********************************/
	   IloModel mp(env); //the master problem for the column generation
	   IloObjective routesCost = IloAdd(mp, IloMinimize(env)); //declare the objective of the problem
	   //add the IloRangeArray for the constraints
	   IloRangeArray visited(env);
	   for(int i=0; i<input.vehNum; i++)
	   {
		   visited.add(IloRange(env, 1, 1 ));
	   }//endfor
	   mp.add(visited);

	   //add the variable and coefficients for the objective and constraints
	   IloNumVarArray paths(env); //declare the decision variables
	   for(int p=0; p<pathSet.size(); p++)
	   {
		   IloNumColumn yInitialColumn = routesCost(costPathSet.at(p));
		   for(int i=0; i<input.vehNum; i++)
		   {
			   yInitialColumn += visited[i](aPathSet.at(p).at(i));
		   }
		   paths.add(IloNumVar(yInitialColumn, 0, IloInfinity));
	   }//endfor

	   //instance the master problem model
	   IloCplex mpSolver(mp);

	   /********************************/
	   /*regarding the pricing problem */
	   /********************************/
	    IloModel sbp(env);
	    char VarName[24];
		int num = input.dimension;

		//declare and initialize the decision variables
		IntVarMatrix xVars(env,num);
		for(int i=0; i<num; i++)
		{
			xVars[i]=IloIntVarArray(env, num);
			for(int j=0; j<num; j++)
			{
				sprintf_s(VarName, "x_%d%d",i,j);
				xVars[i][j]=IloIntVar(env, 0, 1, VarName);			
			}//endforj
		}//endfori
		IloNumVarArray sTime(env, num+1);
		for(int i=0; i<num+1; i++)
		{
			sprintf_s(VarName, "s_%d",i);
			sTime[i]=IloNumVar(env, 0, IloInfinity, VarName);
		}

		//declare the expressions used for the constraints
		IloExprArray flow(env, num+1);
		for(int i=0; i<num+1; i++)
		{
			flow[i]=IloExpr(env);
		}//endfori
		

		//flow conservation
		for(int i=1; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				flow[i]+=xVars[i][j]-xVars[j][i];
			}//endforj
			sbp.add(flow[i]==0);
			sbp.add(xVars[i][i]==0);
		}//endfori
		sbp.add(xVars[0][0]==0);
		for(int j=0; j<num; j++)
		{
			flow[0]+=xVars[0][j];
			flow[num]+=xVars[j][0];
		}
		sbp.add(flow[0]==1);
		sbp.add(flow[num]==1);

		//the capacity constraints
		IloExpr cap(env);
		for(int i=1; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				cap+=input.demand.at(i)*xVars[i][j];
			}//endforj
		}//endfori
		sbp.add(cap<=input.capacity);

		//the time windows constraints
		for(int i=0; i<num; i++)
		{
			sbp.add(sTime[i]>=input.rt.at(i));
			sbp.add(sTime[i]<=input.dt.at(i));
			for(int j=0; j<num; j++)
			{
				if(j!=0)
					sbp.add(sTime[i]+input.st.at(i)-M*(1-xVars[i][j])<=sTime[j]);
				else
					sbp.add(sTime[i]+input.st.at(i)-M*(1-xVars[i][j])<=sTime[num]);
			}//endforj
		}//endfori
		sbp.add(sTime[num]>=input.rt.at(0));
		sbp.add(sTime[num]<=input.dt.at(0));
		IloCplex sbpSolver(sbp);


		/*******************************************/
	   /*regarding the column generation framework */
	   /********************************************/
		double rc=-1;//some initial value
		while(rc<-1*smallNum)
		{
			mpSolver.solve();//solve the master problem
			env.out()<< "master problem obj value: "<< mpSolver.getObjValue()<<endl;

			//obtain the dual variables
			IloNumArray price(env, input.vehNum);
			env.out()<< "dual variable values: "<<endl;
			for(int i=0; i<input.vehNum; i++)
			{
				price[i]=mpSolver.getDual(visited[i]);
				env.out()<<price[i]<<" ";
			}//endfor	

			//obtain the new cost
			vector<vector<double>> newCost;
			for(int i=0; i<num; i++)
			{
				vector<double> newCostUnit;
				for(int j=0; j<num; j++)
				{
					if(i>=1)
						newCostUnit.push_back( input.cost.at(i).at(j) - price[i-1] );
					else
						newCostUnit.push_back( input.cost.at(i).at(j) );
				}
				newCost.push_back(newCostUnit);
			}//endfori

			//add the objective for sbp
			IloObjective reducedCost = IloMinimize(env);
			IloExpr obj(env);
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					if(i!=j)
					{
						obj+=newCost.at(i).at(j)*xVars[i][j];
					}//endif
				}//endforj
			}//endfori
			reducedCost.setExpr(obj);
			sbp.add(reducedCost);

			//export the model
			sbpSolver.exportModel("sbp.lp");
			//solve the subproblem and retrieve solution
			sbpSolver.solve();
			rc = sbpSolver.getObjValue(); //obtain the objective value	
			env.out()<<"the subproblem objective value is "<<rc<<endl;

			IloNumArray newPath(env, input.dimension);
			vector<int> pathUnit;
			vector<int> newPathAp(input.vehNum, 0);
			double newPathCost = 0;
			for(int i=0; i<num; i++)
			{
				sbpSolver.getValues(newPath, xVars[i]);
				for(int j=0; j<num; j++)
				{
					if(abs(newPath[j]-1)<smallNum)
					{
						if(i!=0)
						{
							pathUnit.push_back(i);
							newPathAp.at(i-1)+=1;
						}
						newPathCost += input.cost.at(i).at(j);
					}//endif
				}//endforj
			}//endfori
			pathSet.push_back(pathUnit);
			aPathSet.push_back(newPathAp);
			costPathSet.push_back(newPathCost);

			//adding the column to master problem
		   IloNumColumn newColumn = routesCost(newPathCost);
		   for(int i=0; i<input.vehNum; i++)
		   {
			   newColumn += visited[i](newPathAp.at(i));
		   }
		   paths.add(IloNumVar(newColumn, 0, IloInfinity));

		   //end the lower level problem objective
		   obj.end();
		   reducedCost.end();
		}//endwhile

		/*********not sure why IloConversion is not working here*/
		/*
		IloInt pSize = paths.getSize();
		for(IloInt i=0; i<pSize; i++)
		{
			mp.add(paths[i]<=1);
			IloConversion c=IloConversion(env, paths[i], ILOINT);
			mp.add(c);
		}
		//export the model
		mpSolver.exportModel("mp.lp");

		mpSolver.solve();
		double earlyBranchCost = mpSolver.getObjValue();
		env.out()<<"early brancing result: "<<earlyBranchCost<<endl;*/

	   //reconstruct the model to solve the problem
		IloModel eb(env); //the early branching approach
	   IloObjective routesCostEb = IloAdd(eb, IloMinimize(env)); //declare the objective of the problem
	   //add the IloRangeArray for the constraints
	   IloRangeArray visitedEb(env);
	   for(int i=0; i<input.vehNum; i++)
	   {
		   visitedEb.add(IloRange(env, 1, 1 ));
	   }//endfor
	   eb.add(visitedEb);

	   //add the variable and coefficients for the objective and constraints
	   IloIntVarArray pathsEb(env); //declare the decision variables
	   for(int p=0; p<pathSet.size(); p++)
	   {
		   IloNumColumn yInitialColumnEb = routesCostEb(costPathSet.at(p));
		   for(int i=0; i<input.vehNum; i++)
		   {
			   yInitialColumnEb += visitedEb[i](aPathSet.at(p).at(i));
		   }
		   paths.add(IloIntVar(yInitialColumnEb, 0, 1));
	   }//endfor

	   //instance the master problem model
	   IloCplex ebSolver(eb);
	   ebSolver.solve();
	   ebSolver.exportModel("eb.lp");
	   env.out()<<ebSolver.getObjValue()<<endl;
     
   }//endtry
   catch (IloException& ex) {
      cerr << "Error: " << ex << endl;
   }//endcatch
   catch (...) {
      cerr << "Error" << endl;
   }//endcatch

   env.end();
   return 0;
}