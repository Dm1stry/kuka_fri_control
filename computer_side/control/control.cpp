#include "control.hpp"

using namespace kuka_control;

Control::Control(const double Kp, const double Kd, const double v_max, const double time_tick):
Kp_(Kp),
Kd_(Kd),
time_tick_(time_tick),
q_previous_(0),
v_max_(v_max),
torque_max_(10),
k_(1),
lambda_(1)
{
    I_h_ = 0;
    b_h_ = 0;
    T_h_ = 0;

    lambda_ = 1;
    k_ = 1;
}

void Control::setPreviousPos(double q)
{
    q_previous_ = q;
}

double Control::calcTorque(double q, double q_d)
{
    v_ = (q - q_previous_)/time_tick_;
    q_previous_ = q;

    s_ = -v_ + lambda_*(q_d-q);

    I_h_ += gamma_*s_*lambda_*v_;
    b_h_ += -gamma_*v_*s_;
    T_h_ += -gamma_*s_;

    torque_ = -I_h_*lambda_*v_ - b_h_*v_ - T_h_ + k_*s_;

    torque_ = std::clamp(torque_, -torque_max_, torque_max_);

    return torque_;
}

int Control::sat(double s)
{
    if (std::abs(s) > 0.05) return k_*sign(s);
    else return s/0.05;
}

int kuka_control::sign(double a)
{
    if (a > 0.000001) return 1;
    else if (a < -0.000001) return -1;
    else return 0;
}