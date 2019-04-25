#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  prev_cte = 0.0;
  del_cte = 0.0;
  sum_cte = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  // It is for p_term
  p_error = cte * Kp;
  
  // It is for d_term
  del_cte = cte - prev_cte;
  prev_cte = cte;
  d_error = del_cte * Kd;
  
  // It is for i term
  sum_cte += cte;
  i_error = sum_cte * Ki;
  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return - p_error - i_error - d_error;
}