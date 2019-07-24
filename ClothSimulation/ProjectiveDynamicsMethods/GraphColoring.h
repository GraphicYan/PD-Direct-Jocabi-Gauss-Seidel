#pragma once
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <Eigen\Sparse>
#include <set>
#include<time.h>
using namespace std;

const int SHRINKING = 1;
vector<vector<int>> randomizedGraphColoring(Eigen::MatrixXf& A);
vector<vector<int>> randomizedGraphColoring(Eigen::MatrixXf& A, Eigen::VectorXi& indices, Eigen::VectorXf& nodes);
