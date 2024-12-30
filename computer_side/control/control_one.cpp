#include "control_one.hpp"

using namespace controller_one_joint;

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
    torque_ = 0;
    
    I_h_ = 0.002;
    b_h_ = 0.1;
    T_h_ = 0.0;
    mr_h_ = 0.0;

    gamma_ = 1;
    lambda_ = 3;
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

    if (abs(q_d-q)>0.01745)
    {
        I_h_ += -gamma_*s_*lambda_*v_*time_tick_;
        b_h_ += -gamma_*v_*s_*time_tick_;
        T_h_ += 10*gamma_*s_*sign(q_d-q)*time_tick_;
        // mr_h_ += gamma_*s_*sin(q-M_PI_4);
    }
    else
    {
        T_h_ = 0.0;
    }

    T_h_ = std::clamp(T_h_, -1., 1.);

    std::cout << I_h_ << "\t" << b_h_ << "\t" << T_h_ << "\t" << mr_h_ << "\n";

    torque_ = -I_h_*lambda_*v_ + b_h_*v_ + T_h_*sign(q_d-q) + mr_h_*sin(q-M_PI_4) + k_*s_;

    torque_ = std::clamp(torque_, -torque_max_, torque_max_);

    return torque_;
}

int Control::sat(double s)
{
    if (std::abs(s) > 0.05) return k_*sign(s);
    else return s/0.05;
}

int controller_one_joint::sign(double a)
{
    if (a > 0.000001) return 1;
    else if (a < -0.000001) return -1;
    else return 0;
}