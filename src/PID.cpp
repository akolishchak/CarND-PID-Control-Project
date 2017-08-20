#include "PID.h"
#include <iostream>
#include <math.h>
#include <limits>
using namespace std;

/**
 * PID class constructor
 *
 */
PID::PID() : Kp_(params_[0]), Ki_(params_[1]), Kd_(params_[2])
{
    //
    // initialize error terms and velocity
    //
    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;
    velocity_ = 0;

    first_pass_ = true;

    //
    // initialize Twiddle variables
    //
    optim_idx_ = 0;
    optim_phase_ = 1;
    optim_step_ = 0;
    error_sum_ = 0;
    best_error_sum_ = numeric_limits<double>::max();

    //
    // Initialize gains
    //
    Init(0, 0, 0);
}

PID::~PID()
{

}

/**
 * Initialize PID controller gains
 * @param Kp - proportional gain
 * @param Ki - integral gain
 * @param Kd - derivative gain
 * @param Dp - initial learning rate for proportional gain (Twiddle)
 * @param Di - initial learning rate for integral gain (Twiddle)
 * @param Dd - initial learning rate for derivative gain (Twiddle)
 */
void PID::Init(double Kp, double Ki, double Kd, double Dp, double Di, double Dd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    optim_learning_rate_[0] = Dp;
    optim_learning_rate_[1] = Di;
    optim_learning_rate_[2] = Dd;
}

/**
 * Update controller terms
 *
 * @param cte       - cross track error
 * @param velocity  - normalized vehicle velocity
 */
void PID::UpdateError(double cte, double velocity)
{
    if ( first_pass_ ) {
        // init p_error_ to avoid d_error_ spike
        p_error_ = cte;
        first_pass_ = false;
    }
    //
    // derivative term
    //
    d_error_ = cte - p_error_;
    //
    // proportional term
    //
    p_error_ = cte;
    //
    // integral term
    //
    i_error_ += cte;
    //
    // adjust for small velocity
    //
    velocity_ = velocity > 1e-3 ? velocity : 1e-3;
}

/**
 * Get controlling signal as total error
 *
 * @return
 */
double PID::TotalError()
{
    double error = p_error_ * Kp_ / velocity_ + d_error_ * Kd_ + i_error_ * Ki_;

    return error;
}

/**
 * Twiddle optimization step to tune the controller gains
 *
 * @param cte - cross track errror
 */
void PID::Twiddle(double cte)
{
    // sum up CTEs
    error_sum_ += sqrt(cte*cte);
    // optimize on each 5000s step, to get errors for entire track
    if ( ++optim_step_ >= 5000 ) {

        switch ( optim_phase_ ) {
            case 1:
                if ( error_sum_ < best_error_sum_ ) {
                    best_error_sum_ = error_sum_;
                    optim_learning_rate_[optim_idx_] *= 1.1;
                    optim_phase_ = 0;
                } else {
                    params_[optim_idx_] -= 2 * optim_learning_rate_[optim_idx_];
                    optim_phase_++;
                }
                break;

            case 2:
                if ( error_sum_ < best_error_sum_ ) {
                    best_error_sum_ = error_sum_;
                    optim_learning_rate_[optim_idx_] *= 1.1;
                } else {
                    params_[optim_idx_] += optim_learning_rate_[optim_idx_];
                    optim_learning_rate_[optim_idx_] *= 0.9;
                }
                optim_phase_ = 0;
                break;
        }

        if ( optim_phase_ == 0 ) {
            // switch to the next parameter
            do {
                optim_idx_++;
                if ( optim_idx_ > 2 )
                    optim_idx_ = 0;
            } while ( optim_learning_rate_[optim_idx_] == 0 );

            params_[optim_idx_] += optim_learning_rate_[optim_idx_];
            optim_phase_++;
        }

        error_sum_ = 0;
        optim_step_ = 0;
    }
}