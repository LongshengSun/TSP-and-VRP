/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: tspSolber.cpp                                                */
/*Description: regarding the implemention for solving TSP            */
/*             problem                                                */
/**********************************************************************/
#include <ilcplex/ilocplex.h>
#include <vector>
using std::vector;
#include <iostream>
using namespace std;

#include "tspSolver.h"


/*****************************************************************************/
/* using quadratic formulation to solve TSP                                  */
/*****************************************************************************/
double QF(const vector<vector<double>> cost, double &gap, bool flag, int num)
{
	//the optimizatio model
	try {
		//declare the environment
		IloEnv env;
		//declare the model
		IloModel model(env);

		//used for recording variables name
		char VarName[24];

		NumVarMatrix xVars(env,num);
		for(int i=0; i<num; i++)
		{
			xVars[i]=IloNumVarArray(env, num);
			for(int s=0; s<num; s++)
			{
				sprintf_s(VarName, "x_%d%d",i+1,s+1);
				if(flag)
				{
					xVars[i][s]=IloNumVar(env, 0, 1, IloNumVar::Int, VarName);
				}
				else
				{
					xVars[i][s]=IloNumVar(env, 0, 1, VarName);
				}
				
			}//endforj
		}//endfori

		NumVar3D wVars(env, num);
		for(int s=0; s<num; s++)
		{
			wVars[s]=NumVarMatrix(env,num);
			for(int i=0; i<num; i++)
			{
				wVars[s][i]=IloNumVarArray(env, num);
				for(int j=0; j<num; j++)
				{
					sprintf_s(VarName, "w_%d%d%d",s+1, i+1, j+1);
					wVars[s][i][j]=IloNumVar(env, 0, 1, VarName);
				}//endforj
			}//endfori
		}//endfors

		

		//declare the expressions used for the constraints
		IloExprArray exprXiSum(env, num);
		IloExprArray exprXsSum(env, num);
		for(int i=0; i<num; i++)
		{
			exprXiSum[i]=IloExpr(env);
		}//endfori
		for(int s=0; s<num; s++)
		{
			exprXsSum[s]=IloExpr(env);
		}//endfors

		IloCplex cplex(model); //instance the model
		IloExpr obj(env);	// the objective expression
		for(int s=0; s<num-1; s++)
		{
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					if(i!=j)
					{
						obj+=cost.at(i).at(j)*wVars[s][i][j];
					}
				}//endforj
			}//endfori
		}//endfor

		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					obj+=cost.at(i).at(j)*wVars[num-1][i][j];
				}
			}//endforj
		}//endfori

		model.add(IloMinimize(env, obj));

		for(int i=0; i<num; i++)
		{
			for(int s=0; s<num; s++)
			{
				exprXiSum[i]+=xVars[i][s];
				exprXsSum[i]+=xVars[s][i];
			}//endforj
			model.add(exprXiSum[i]==1);
			model.add(exprXsSum[i]==1);
		}//endfori

		for(int s=0; s<num-1; s++)
		{
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					if(i!=j)
					{
						model.add(wVars[s][i][j]>=xVars[i][s]+xVars[j][s+1]-1);
					}
				}//endforj
			}//endfori
		}//endfor

		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					model.add(wVars[num-1][i][j]>=xVars[i][num-1]+xVars[j][0]-1);
				}
			}//endforj
		}//endfori

		IloExprArray lhs(env);
		IloNumArray  rhs(env);
		cplex.use(CtCallback(env, lhs, rhs, cplex.getParam(IloCplex::EpRHS)));

		
		cplex.setParam(IloCplex::TiLim, 600);
		cplex.exportModel("model.lp");
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize the problem" << endl;
			throw(-1);
		}
		

		env.out() <<"Solutioin status = "<<cplex.getStatus()<<endl;
		env.out() <<"Solutioin object value = "<<cplex.getObjValue()<<endl;

		if(flag)
		{
			gap=cplex.getMIPRelativeGap();
		}

		double objective=cplex.getObjValue();

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

/*****************************************************************************/
/* using Miller-Tucker-Zemlin formulation to solve TSP                       */
/*****************************************************************************/
double MTZ(const vector<vector<double>> cost, double & gap, bool flag, int num)
{
	//the optimizatio model
	try {
		//declare the environment
		IloEnv env;
		//declare the model
		IloModel model(env);

		//declare the decision variables for ui
		IloNumVarArray uVars(env, num);
		char VarName[24];
		for(int i=0; i<num; i++)
		{
			sprintf_s(VarName, "u_%d",i+1);
			uVars[i]=IloNumVar(env, 0, +IloInfinity, VarName);
		}//endfor

		NumVarMatrix xVars(env,num);
		for(int i=0; i<num; i++)
		{
			xVars[i]=IloNumVarArray(env, num);
			for(int j=0; j<num; j++)
			{
				sprintf_s(VarName, "x_%d%d",i+1,j+1);
				if(flag)
				{
					xVars[i][j]=IloNumVar(env, 0, 1, IloNumVar::Int, VarName);
				}
				else
				{
					xVars[i][j]=IloNumVar(env, 0, 1, VarName);
				}
				
			}//endforj
		}//endfori

		//declare the expressions used for the constraints
		IloExprArray exprXiSum(env, num);
		IloExprArray exprXjSum(env, num);
		for(int i=0; i<num; i++)
		{
			exprXiSum[i]=IloExpr(env);
		}//endfori
		for(int j=0; j<num; j++)
		{
			exprXjSum[j]=IloExpr(env);
		}//endfors

		IloCplex cplex(model); //instance the model
		IloExpr obj(env);	// the objective expression

		//obtain the objective
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					obj+=cost.at(i).at(j)*xVars[i][j];
				}//endif
			}//endforj
		}//endfori
		model.add(IloMinimize(env, obj));

		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					exprXiSum[i]+=xVars[i][j];
					exprXjSum[i]+=xVars[j][i];
				}//endif
				if(i!=0 && j!=0)
				{
					model.add(uVars[i]-uVars[j]+num*xVars[i][j]<=num-1);
				}//endif
			}//endforj
			model.add(exprXiSum[i]==1);
			model.add(exprXjSum[i]==1);
		}//endfori

		cplex.setParam(IloCplex::TiLim, 600);
		cplex.exportModel("model.lp");
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize the problem" << endl;
			throw(-1);
		}
		

		env.out() <<"Solutioin status = "<<cplex.getStatus()<<endl;
		env.out() <<"Solutioin object value = "<<cplex.getObjValue()<<endl;

		if(flag)
		{
			gap=cplex.getMIPRelativeGap();
		}

		double objective=cplex.getObjValue();

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


/*****************************************************************************/
/* using multicommodity flow formulation to solve TSP                       */
/*****************************************************************************/
double MFF(const vector<vector<double>> cost, double &gap, bool flag, int num)
{
	//the optimizatio model
	try {
		//declare the environment
		IloEnv env;
		//declare the model
		IloModel model(env);

		//used for recording name of variable
		char VarName[24];

		NumVarMatrix xVars(env,num);
		for(int i=0; i<num; i++)
		{
			xVars[i]=IloNumVarArray(env, num);
			for(int j=0; j<num; j++)
			{
				sprintf_s(VarName, "x_%d%d",i+1,j+1);
				if(flag)
				{
					xVars[i][j]=IloNumVar(env, 0, 1, IloNumVar::Int, VarName);
				}
				else
				{
					xVars[i][j]=IloNumVar(env, 0, 1, VarName);
				}
				
			}//endforj
		}//endfori

		NumVar3D yVars(env, num);
		for(int k=2; k<=num; k++)
		{
			yVars[k-2]=NumVarMatrix(env,num);
			for(int i=0; i<num; i++)
			{
				yVars[k-2][i]=IloNumVarArray(env, num);
				for(int j=0; j<num; j++)
				{
					sprintf_s(VarName, "y_%d%d",k, i+1, j+1);
					yVars[k-2][i][j]=IloNumVar(env, 0, 1, VarName);
				}//endforj
			}//endfori
		}//endfors

		//declare the expressions used for the constraints
		IloExprArray exprXiSum(env, num);
		IloExprArray exprXjSum(env, num);
		for(int i=0; i<num; i++)
		{
			exprXiSum[i]=IloExpr(env);
		}//endfori
		for(int j=0; j<num; j++)
		{
			exprXjSum[j]=IloExpr(env);
		}//endfors

		ExprMatrix exprMF(env, num-1);
		for(int k=2; k<=num; k++)
		{
			exprMF[k-2]=IloExprArray(env, num);
			for(int i=0; i<num; i++)
			{
				exprMF[k-2][i]=IloExpr(env);
			}//endfori
		}//endfork

		IloCplex cplex(model); //instance the model
		IloExpr obj(env);	// the objective expression

		//obtain the objective
		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					obj+=cost.at(i).at(j)*xVars[i][j];
				}//endif
			}//endforj
		}//endfori
		model.add(IloMinimize(env, obj));

		for(int i=0; i<num; i++)
		{
			for(int j=0; j<num; j++)
			{
				if(i!=j)
				{
					exprXiSum[i]+=xVars[i][j];
					exprXjSum[i]+=xVars[j][i];
				}//endif
			}//endforj
			model.add(exprXiSum[i]==1);
			model.add(exprXjSum[i]==1);
		}//endfori

		vector<vector<int>> dki;
		for(int k=2; k<=num; k++)
		{
			dki.push_back( vector<int> (num, 0) );
		}
		for(int k=2; k<=num; k++)
		{
			for(int i=1; i<=num; i++)
			{
				if(i==1)
				{
					dki.at(k-2).at(i-1)=1;
				}
				else if (i==k)
				{
					dki.at(k-2).at(i-1)=-1;
				}
			}//endfori
		}//endfork

		for(int k=2; k<=num; k++)
		{
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					exprMF[k-2][i]+=yVars[k-2][i][j]-yVars[k-2][j][i];
					if(i!=j)
					{
						model.add(yVars[k-2][i][j]<=xVars[i][j]);
					}
				}//endforj
				model.add(exprMF[k-2][i]==dki[k-2][i]);
			}//endfori
		}//endfork

		cplex.setParam(IloCplex::TiLim, 600);
		cplex.exportModel("model.lp");
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize the problem" << endl;
			throw(-1);
		}
		

		env.out() <<"Solutioin status = "<<cplex.getStatus()<<endl;
		env.out() <<"Solutioin object value = "<<cplex.getObjValue()<<endl;

		if(flag)
		{
			gap=cplex.getMIPRelativeGap();
		}

		double objective=cplex.getObjValue();

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

/*****************************************************************************/
/* using shortest path formulation to solve TSP                      */
/*****************************************************************************/
double SPF(const vector<vector<double>> cost, double &gap, bool flag, int num)
{
	//the optimizatio model
	try {
		//declare the environment
		IloEnv env;
		//declare the model
		IloModel model(env);

		//used for recording name of variable
		char VarName[24];

		NumVar3D xVars(env, num);
		for(int t=1; t<=num; t++)
		{
			xVars[t-1]=NumVarMatrix(env,num);
			for(int i=1; i<=num; i++)
			{
				xVars[t-1][i-1]=IloNumVarArray(env, num);
				for(int j=1; j<=num; j++)
				{
					sprintf_s(VarName, "x_%d%d%d",t, i, j);
					if(flag)
					{
						xVars[t-1][i-1][j-1]=IloNumVar(env, 0, 1, IloNumVar::Int, VarName);
					}//endif
					else
					{
						xVars[t-1][i-1][j-1]=IloNumVar(env, 0, 1, VarName);
					}//endelse
				}//endforj
			}//endfori
		}//endfors

		IloExpr _1Sum(env); //used for the sum for t=1
		IloExpr nSum(env); //used for the sum for n
		IloExprArray _2Sum(env, num); //used for the sum for t=2
		IloExprArray n1Sum(env, num); //used for the sum for n-1
		ExprMatrix flow(env, num-3); //used for the sum for i \in N, t = 3...n-1
		IloExprArray ac(env, num-1); //some additionally constraints

		for(int i=2; i<=num; i++)
		{
			ac[i-2]=IloExpr(env);
		}//endfori

		for(int t=3; t<=num-1; t++)
		{
			flow[t-3]=IloExprArray(env, num);
			for(int i=1; i<=num; i++)
			{
				flow[t-3][i-1]=IloExpr(env);
			}//endfori
		}//endfort
		for(int i=1; i<=num; i++)
		{
			_2Sum[i-1]=IloExpr(env);
			n1Sum[i-1]=IloExpr(env);
		}//endfori

		

		IloCplex cplex(model); //instance the model
		IloExpr obj(env);	// the objective expression

		//obtain the objective
		for(int t=0; t<num; t++)
		{
			for(int i=0; i<num; i++)
			{
				for(int j=0; j<num; j++)
				{
					if(i!=j)
					{
						obj+=cost.at(i).at(j)*xVars[t][i][j];
					}
				}
			}//endfori
		}//endfort
		model.add(IloMinimize(env, obj)); //add the objective


		//adding the constraints
		for(int j=1; j<num; j++)
		{
			_1Sum+=xVars[0][0][j];
		}//endforj
		model.add(_1Sum==1);

		for(int i=0; i<num; i++)
		{
			for(int j=1; j<num; j++)
			{
				if(i!=j)
				{
					_2Sum[i]+=xVars[1][i][j];
				}
			}
			_2Sum[i]-=xVars[0][0][i];
			model.add(_2Sum[i]==0);
		}//endfori

		for(int t=3; t<=num-1; t++)
		{
			for(int i=1; i<=num; i++)
			{
				for(int j=2; j<=num; j++)
				{
					if(i!=j)
					{
						flow[t-3][i-1]+=xVars[t-1][i-1][j-1];
						flow[t-3][i-1]-=xVars[t-2][j-1][i-1];
					}//endif
				}//endforj
				model.add(flow[t-3][i-1]==0);
			}//endfori
		}//endfort

		for(int i=0; i<num; i++)
		{
			for(int j=1; j<num; j++)
			{
				if(i!=j)
				{
					n1Sum[i]-=xVars[num-2][j][i];
				}
			}
			n1Sum[i]+=xVars[num-1][i][0];
			model.add(n1Sum[i]==0);
		}//endfori

		for(int i=1; i<num; i++)
		{
			nSum+=xVars[num-1][i][0];
		}//endforj
		model.add(nSum==1);

		for(int i=2; i<=num; i++)
		{
			for(int t=2; t<=num-1; t++)
			{
				for(int j=2; j<=num; j++)
				{
					if(i!=j)
					{
						ac[i-2]+=xVars[t-1][i-1][j-1];
					}//endif
				}//endforj
			}//endfort
			ac[i-2]+=xVars[num-1][i-1][0];
			model.add(ac[i-2]<=1);
		}//endfori
		

		cplex.setParam(IloCplex::TiLim, 600);
		cplex.exportModel("model.lp");
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize the problem" << endl;
			throw(-1);
		}
		

		env.out() <<"Solutioin status = "<<cplex.getStatus()<<endl;
		env.out() <<"Solutioin object value = "<<cplex.getObjValue()<<endl;

		if(flag)
		{
			gap=cplex.getMIPRelativeGap();
		}

		double objective=cplex.getObjValue();

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