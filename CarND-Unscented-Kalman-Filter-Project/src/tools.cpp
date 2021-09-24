#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    
    VectorXd temp(4);
    VectorXd rtemp(4);
    rtemp << 0,0,0,0;
    int n = 0;
    
    for(int i = 0; i < estimations.size(); i++){
        temp = estimations[i] - ground_truth[i];
        for(int i = 0; i < temp.size(); i++){
            rtemp(i) = temp(i)*temp(i) +rtemp(i);
        }
        n++;
    }
    
    for(int i = 0; i < temp.size(); i++){
        rtemp(i) = pow((rtemp(i)/n),.5);
    }
    
    return rtemp;
}
