#include "PID.h"
using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;


}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  i_error = i_error + cte;
  d_error = cte - p_error;
  p_error = cte;
  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double tot_error;
  tot_error = -Kp * p_error - Ki * i_error - Kd * d_error;
  return tot_error;  // TODO: Add your total error calc here!
}

/// TRYING TO MAKE TWEDDLE ALGORITHM HERW
double * calc_gains_PID(double tol){
  static double  p[3] = {0,0,0};
  double dp[3] = {1,1,1};
  return p;
}


