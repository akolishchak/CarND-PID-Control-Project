# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Writeup

In this project I have implemented a PID controller to maneuver the vehicle around the Udacity Simulator track.

PID controller is a weighted sum of three terms, such as an error with respect to target variable, derivative of that error and its integral. In this project the terms are:
 * Vehicle position cross track error (CTE),
 * Sum of CTEs,
 * Difference between current and previous CTEs.

The weights are:
 * Kp - weight of CTE, or proportional gain;
 * Ki - weight of sum of CTEs, or integral gain;
 * Kd - weight of difference between current and previous CTEs, or derivative gain.

I use negative output value of the controller as a steering signal to reduce CTE.

To select Kp, Ki, and Kd gains I performed manual tuning. Initially, I set all gains to zero and started to increase proportional gain, Kp. The value of 0.1 allowed vehicle to complete the track, however its trajectory oscillated due to overshooting of steering signal. Then I started to increase derivative gain, Kd, to reduce the oscillation caused by overshooting of proportional gain. Finally, I tried to adjust the integral gain, Ki, that should address vehicle drift. This didn't work well because the integral term may increase or decrease for a long period of time depending on the track curvature. This may work only with very small Ki (< 1e-4) making its effect neglectable, so I set Ki to zero.

I also tried to fine tune manual findings by twiddle (PID::Twiddle() method of PID class, PID.cpp, lines 11-157), but it often led to gains that throw vehicle outside of the track.

I found that controller gains depend on the vehicle speed, so I added a normalized speed related term to the proportional term as CTE * Kp / (speed * 0.01) (main.cpp, line 59). This improves controlling signal but is still not enough to eliminate oscillations.

To control speed I apply throttle adjustment proportional to derivative term (main.cpp, line 71).

#### The implemented controller in action
[![PID controller in action](video_image.png)](https://youtu.be/5VlOHSO1lUw)

