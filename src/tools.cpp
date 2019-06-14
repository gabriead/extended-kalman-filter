#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
    VectorXd rmse(4);
  	rmse << 0,0,0,0;
  
    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
      //std::cout << "Invalid estimation or ground_truth data" << std::endl;
      return rmse;
  	}

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  if(fabs(px*px + py*py) < 0.0001)
  {
    //std::cout<< "CalculateJacobian() - Error - Division by zero"<< std::endl;
    return Hj;
  }
   
   double sqrt_px_py_pow_2 = sqrt(pow(px,2) + pow(py,2));
   double px_py_pow_2 = pow(px,2) + pow(py,2);
   double px_py_pow_3_2 = pow(px_py_pow_2, 3/2);
   
   double py_compound_numerator = py*(vx*py-vy*px);
   double px_compound_numerator = px*(vy*px-vx*py);
   
   Hj << px/sqrt_px_py_pow_2, py/sqrt_px_py_pow_2, 0,0,
         -(py/px_py_pow_2)  , px/px_py_pow_2     , 0,0,
         py_compound_numerator/px_py_pow_3_2,px_compound_numerator/px_py_pow_3_2,px/sqrt_px_py_pow_2, py/sqrt_px_py_pow_2;  
  
  return Hj;
}
