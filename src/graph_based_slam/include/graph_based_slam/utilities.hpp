#pragma once

#include <cmath>
#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/types_slam2d.h"


/*
Method that computes the Mahalanobis Distance
@ param z         : measurment of the cone;
@ param optimizer : graph/solver of the problem;
@ param H         : output that represents the Jacobian of the prediction function;
@ param vid       : id of the vertex that rapresents the robot pose;
@ param lid       : id of the vertex that rapresents the landmark pose;
@ return chi2     : mahalanobis distance;
*/
double computeGatingChi2(Eigen::Vector2d z, g2o::SparseOptimizer& optimizer, int vid, int lid);

/*
Method that computes the Jacobian matrix needed for the computation of the gating covariance
@ param xr : vertice rappresentante posa del robot xr;
@ param xl : vertice rappresentante landmark xl;
@ param H  : output that represents the Jacobian of the prediction function;
*/
void computeH(const g2o::VertexSE2* xr, const g2o::VertexPointXY* xl, Eigen::Matrix<double, 2, 5>& H);

/*
Method for computing the gating threshold based on probability of acceptance
@ param P   : how sure you want to be that inliers are not rejected
@ param dim : degrees of freedom of the measurment; ( nel nostro caso 2 )
@ return th : threshold;
*/
double chi2inv(double P, unsigned int dim);

// ----- COSE MATEMATICHE DI SUPPORTO -----
double normalQuantile(double p);
double normalCDF(double u);