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

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	//  * the estimation vector size should not be zero
    if(estimations.size() == 0) {
        cout << "ERROR: estimation vector is zero" << endl;
        return rmse;
    }
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()) {
        cout << "ERROR: estimation vector size not equal to ground truth vector" << endl;
        return rmse;
    }
    
	//accumulate squared residuals

	for(int i=0; i < estimations.size(); ++i){
        VectorXd res = estimations[i] - ground_truth[i];
        res = res.array()*res.array();
        rmse += res;
	}

	//calculate the mean
    rmse = rmse/estimations.size();
	//calculate the squared root
    rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

    float g = px*px+py*py;
    float gsqrt = sqrt(g);
    float g32 = g*gsqrt;

	//check division by zero
	if(fabs(g)<0.00001) {
	    cout << "ERROR: Dividing by zero" << endl;
	    return Hj;
	}
	//compute the Jacobian matrix
    Hj << px/gsqrt, py/gsqrt, 0, 0,
          -py/g, px/g, 0, 0,
          py*(vx*py-vy*px)/g32, px*(vy*px-vx*py)/g32, px/gsqrt, py/gsqrt;
    
	return Hj;
}
