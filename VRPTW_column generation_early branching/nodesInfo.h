/**********************************************************************/
/*Author: Longsheng Sun                                               */
/*File: nodesInfo.h                                                   */
/*Description: reading the coordinates of all the nodes               */
/**********************************************************************/

#ifndef NODEINFO_H
#define NODEINFO_H

#include <vector>
using std::vector;
#include <string>
using std::string;

typedef std::pair<double, double> node;

struct nodesInformation{
	string name;
	double capacity;
	int dimension;
	int vehNum;
	vector<std::pair<double, double>> coordinates;
	vector<double> demand;
	int depot;
	vector<double> rt; //ready time
	vector<double> dt; //due time
	vector<double> st; //service time
	vector<vector<double>> cost;
};


void readNodesFile(char *nodesFile, nodesInformation &input);
void splitString(vector<string> *vec, string *line, char delim);
void calCost(nodesInformation &input);


#endif