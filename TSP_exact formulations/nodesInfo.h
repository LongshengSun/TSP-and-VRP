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


void readNodesFile(char *nodesFile, vector<node>& nodes, int& n);
void splitString(vector<string> *vec, string *line, char delim);
vector<vector<double>> calCost(const vector<node>& nodes, int num);


#endif