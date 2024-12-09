#include "control.hpp"

using namespace kuka_control;

Control::Control(const double Kp, const double Kd, const double Kv, const double v_max, const double time_tick):
Kp_(Kp),
Kd_(Kd),
Kv_(Kv),
time_tick_(time_tick),
q_previous_(0),
v_max_(v_max),
torque_max_(10),
k_(1),
lambda_(1)
{

}

void Control::setPreviousPos(double q)
{
    q_previous_ = q;
}

double Control::calcTorque(double q, double q_d)
{
    v_ = (q - q_previous_)/time_tick_;
    q_previous_ = q;

    double s = -v_ + lambda_*(q_d-q);

    torque_ = 0.01*v_ + 0 - lambda_*v_ + sat(s);

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