/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: vrp.cpp                                                */
/*Description: regarding the implemention for solving VRP            */
/*             problem                                                */
/**********************************************************************/
#include <ilcplex/ilocplex.h>
#include <vector>
using std::vector;
#include <iostream>
#include <functional>
#include <time.h>
using namespace std;

#include "vrp.h"
#include "nodesInfo.h"

/*****************************************************************************/
/* macro to call the callback function                                       */
/*****************************************************************************/
bool addCuts(IntVar3D vars, IloNum eps, nodesInformation & input, IloModel model, IloCplex cplex) 
{
	try{
	   //obtain the current incumbent
	   vector< vector< vector<int> > > solution;
	   for(int i =0; i<vars.getSize(); i++)
	   {
		   vector< vector<int> > solutionUnit;
		   for(int j=0; j<vars[i].getSize(); j++)
		   {
			   vector<int> solutionUnitUnit;
			   for(int k=0; k<K; k++)
			   {
				   solutionUnitUnit.push_back(cplex.getValue(vars[i][j][k]));
			   }
			   solutionUnit.push_back(solutionUnitUnit);
		   }//endforj
		   solution.push_back(solutionUnit);
	   }//endfori


	   IloEnv vrpEnv = model.getEnv();
	   IloExprArray lhs(vrpEnv);
	   IloNumArray  rhs(vrpEnv);
	   //obtain the cuts
	   makeCuts(solution, vars, lhs, rhs, vrpEnv, input);
	    
	   //add the cuts
	   IloInt n = lhs.getSize(); 
	   if(n>=1){
		   for (IloInt i = 0; i < n; i++) {
			  IloNum xrhs = rhs[i];

				 IloRange cut;
				 try {
					cut = (lhs[i] >= xrhs);
					model.add(cut);
				 }
				 catch(const IloException& e){
					cut.end();
					cerr<< "Exception: "<< e<< endl;
				 }
		   }
		   return true;
	   }
		else
			return false;
	}catch(const IloException& e){
		cerr<< "Exception: "<< e<< endl;
	}
}


/*****************************************************************************/
/* construct the lazy constraints                                            */
/*****************************************************************************/
void makeCuts(const vector<vector<vector<int>>>& vars, IntVar3D Dvars, IloExprArray lhs, IloNumArray rhs, IloEnv vrpEnv, nodesInformation &input)
{
	vector< vector<int> > t; //the tours
	t = tours(vars, input);

	for(int m=0; m<t.size(); m++)
	{
		double d = 0;
		IloExpr tourEm(lhs.getEnv());

		int k = t.at(m).at(0);
		for(int i=1; i<t.at(m).size(); i++)
		{
			int nodeTour = t.at(m).at(i);
			d+=input.demand.at(nodeTour);
			for(int j=0; j<input.demand.size(); j++)
			{
				if(!inTour(t.at(m),j))
				{
					for(int mk=0; mk<K; mk++)
						tourEm+=Dvars[nodeTour][j][mk];
				}
			}//endforj
		}//endfori
		lhs.add(tourEm);
		IloNum r = d/input.capacity;
		rhs.add(r);
	}//endform

}

/*****************************************************************************/
/* find the tours                                                            */
/*****************************************************************************/
vector< vector<int> > tours(vector< vector< vector<int> > > vars, nodesInformation &input)
{
	vector< vector<int> > t;
	vector< vector<int> > subTours;

	for(int k=0; k<K; k++)
	{	

		vector<int> subTourUnit;
		vector<int> tUnit;
		tUnit.push_back(k);//the first data denotes the truck k
		tUnit.push_back(0); //push the depot
		subTourUnit.push_back(k);
		for(int j=0; j<vars.at(0).size(); j++)
		{
			if(abs(vars[0][j][k]-1)<smallNum)
			{
				vars[0][j][k]=0;
				tUnit.push_back(j);
				break;
			}
		}//endforj

		

		int cNode = tUnit.at(tUnit.size()-1);
		int depot = input.depot-1;
		bool flag = true;
		while(flag && cNode!=depot)
		{
			flag = false;
			for(int j=0; j<vars.size(); j++)
			{
				if(abs(vars[cNode][j][k]-1)<smallNum)
				{
					vars[cNode][j][k]=0;
					tUnit.push_back(j);
					cNode = j;
					flag = true;
					break;
				}
			}//endforj	
		}//endwhile

		for(int i=0; i<input.dimension; i++)
		{
			for(int j=0; j<input.dimension; j++)
			{
				if(abs(vars[i][j][k]-1)<smallNum)
				{
					subTourUnit.push_back(i);
				}//endif
			}//endj
		}//endi
		
		//add if there is a subtour
		if(subTourUnit.size()>=2)
		{
			subTours.push_back(subTourUnit);
		}
		t.push_back(tUnit);

		
	}//endfork
	
	//return t;
	return subTours;
	
	
}

/*****************************************************************************/
/* detect the subtour elimination nodes                                      */
/*****************************************************************************/
bool inTour(const vector<int>& t, int n)
{
	bool flag = false;
	//starting from 1 since the first is the truck numver
	for(int i=1; i<t.size(); i++)
	{
		if(t.at(i)==n)
		{
			flag = true;
		}
	}

	return flag;
}


/*****************************************************************************/
/* solving the VRP problem                                                   */
/*****************************************************************************/
double vrpSolver(nodesInformation &input)
{
	//the optimizatio model
	try {
		//declare the environment
		IloEnv env;
		//declare the model
		IloModel model(env);

		//declare the decision variables for x
		char VarName[24];
		int num = input.dimension;
		IntVar3D xVars(env,num);
		for(int i=0; i<num; i++)
		{
			xVars[i]=IntVarMatrix(env, num);
			for(int j=0; j<num; j++)
			{		
				xVars[i][j]=IloIntVarArray(env, K);
				for(int k=0; k<K; k++)
				{
					sprintf_s(VarName, "x_%d%d%d",i+1,j+1, k+1);
					xVars[i][j][k]=IloIntVar(env, 0, 1, VarName);
				}//endfork				
			}//endforj
		}//endfori

		IloCplex cplex(model); //instance the model
		
		//addd the objective
		IloExpr obj(env);	// the objective expression
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{		
				for(int k=0; k<K; k++)
				{
					obj+=input.cost.at(i).at(j)*xVars[i][j][k];
				}//endfork				
			}//endforj
		}//endfori
		model.add(IloMinimize(env, obj));


		//declare the expressions used for the constraints
		ExprMatrix flow(env, num+1);
		for(int i=0; i<num+1; i++)
		{
			flow[i]=IloExprArray(env, K);
			for(int k=0; k<K; k++)
			{
				flow[i][k]=IloExpr(env);
			}//endforj
		}//endfori

		//flow conservation
		for(int i=1; i<num; i++)
		{		
			for(int k=0; k<K; k++)
			{
				for(int j=0; j<num; j++)
				{
					flow[i][k] += xVars[i][j][k] - xVars[j][i][k];
				}//endforj
				model.add(flow[i][k]==0);
			}//endk			
		}//endfori

		//leaving and coming only one truck
		for(int k=0; k<K; k++)
		{
			for(int i=0; i<num; i++)
			{
				flow[0][k] += xVars[0][i][k];
				flow[num][k] += xVars[i][0][k];
			}//endforj
			model.add(flow[0][k]==1);
			model.add(flow[num][k]==1);
		}//endfrok

		//capacity constraints
		IloExprArray cap(env, K);
		for(int k=0; k<K; k++)
		{
			cap[k]=IloExpr(env);
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					cap[k]+=input.demand.at(i)*xVars[i][j][k];
				}//endj
			}//endfori
			model.add(cap[k]<=input.capacity);
		}//endfork

		for(int i=0; i<num; i++)
		{
			for(int k=0; k<K; k++)
			{
				model.add(xVars[i][i][k]==0);
			}
		}

		IloExprArray visited(env, num-1);
		for(int i=0; i<num-1; i++)
		{
			visited[i]=IloExpr(env);
		}

		//each node musted be visted
		for(int i=1; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				for(int k=0; k<K; k++)
				{
					visited[i-1]+=xVars[i][j][k];
				}
			}//endforj
			model.add(visited[i-1]==1);
		}//endfori




		cplex.setParam(cplex.TiLim, 600);
		
		double time=0;
		bool flag = true;
		double objective=0;
		while(flag && time < 600)
		{
			clock_t start, end;
			start=clock();	
			cplex.exportModel("model.lp");
			if ( !cplex.solve() ) {
				env.error() << "Failed to optimize the problem" << endl;
				throw(-1);
			}
			env.out() <<"Solutioin status = "<<cplex.getStatus()<<endl;
			env.out() <<"Solutioin object value = "<<cplex.getObjValue()<<endl;
			objective=cplex.getObjValue();
			//obtain the current incumbent
		   vector< vector< vector<int> > > solution;
		   for(int i =0; i<xVars.getSize(); i++)
		   {
			   vector< vector<int> > solutionUnit;
			   for(int j=0; j<xVars[i].getSize(); j++)
			   {
				   vector<int> solutionUnitUnit;
				   for(int k=0; k<K; k++)
				   {
					   solutionUnitUnit.push_back(cplex.getValue(xVars[i][j][k]));
				   }
				   solutionUnit.push_back(solutionUnitUnit);
			   }//endforj
			   solution.push_back(solutionUnit);
		   }//endfori

		   for(int i=0; i<input.dimension; i++)
			{
				for(int j=0; j<input.dimension; j++)
				{
					if(abs(solution[i][j][0]-1)<smallNum)
					{
						env.out()<< i << " "<< j << endl;
					}//endif
				}//endj
			}//endi
			flag=addCuts(xVars, cplex.getParam(IloCplex::EpRHS),input, model, cplex);

			end=clock();
			time=time+(double) (end-start)/CLOCKS_PER_SEC;
		}
		
		model.end();
		env.end();

		return objective;
			
	}///endtry
	catch (std::exception& e) { 
		cerr << "Instance of std::exception caught: " << e.what() << endl; 
	}//endcatch
	catch (...) {
		cerr<<"Unknown exception caught"<<endl;
	}//endcatch
}


