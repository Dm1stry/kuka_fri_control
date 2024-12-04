#include "control.hpp"

using namespace kuka_control;

Control::Control(const double Kp, const double Kd, const double Kv, const double time_tick, const double static_friction, const double dynamic_friction):
Kp_(Kp),
Kd_(Kd),
Kv_(Kv),
time_tick_(time_tick),
static_friction_(static_friction),
dynamic_friction_(dynamic_friction),
q_previous_(0)
// torque_(NUM_J)
{

}

double Control::calcTorque(double q, double q_d)
{
    v_ = (q - q_previous_)/time_tick_;
    q_previous_ = q;

    if (v_ <= 0.001)
    {
        torque_ = Kp_*(q_d - q) - Kd_*v_ + static_friction_;
    }
    else
    {
        torque_ = Kp_*(q_d - q) - Kd_*v_ + dynamic_friction_*(v_/abs(v_));
    }

    return torque_;
}