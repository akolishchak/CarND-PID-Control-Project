#ifndef PID_H
#define PID_H

class PID {
public:
    /*
    * Errors
    */

    double p_error_;
    double i_error_;
    double d_error_;

    double velocity_;

    /*
    * Coefficients
    */
    double &Kp_;
    double &Ki_;
    double &Kd_;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double Dp = 0, double Di = 0, double Dd = 0);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte, double velocity);

    /*
     * Twiddle optimization step
     */
    void Twiddle(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

private:
    //
    // Store gains in array for twiddle
    //
    double params_[3];

    //
    // twiddle variables
    //
    int optim_idx_;
    int optim_phase_;
    int optim_step_;
    double optim_learning_rate_[3];
    double error_sum_;
    double best_error_sum_;

    bool first_pass_;
};

#endif /* PID_H */
