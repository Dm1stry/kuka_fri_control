#include "control_full.hpp"

using namespace controller;

Control::Control(std_jarray v_max, const double time_tick):
time_tick_(time_tick)
{
    if (v_max.size() == 7)
    {
        v_max_ << v_max[0], v_max[1], v_max[2], v_max[3], v_max[4], v_max[5], v_max[6];
    }
    else
    {
        v_max_ << 25, 25, 25, 25, 25, 25, 25;
    }
    pinocchio::urdf::buildModel("robot/iiwa.urdf", model_);
    data_ = pinocchio::Data(model_);
}

bool Control::setPreviousPos(const std_jarray& q)
{
    if (q.size() == 7)
    {
        previous_q_ << q[0], q[1], q[2], q[3], q[4], q[5], q[6];
        return true;
    }
    return false;
}

void Control::calcVel()
{
    dq_ = (q_ - previous_q_)/time_tick_;
    previous_q_ = q_;
}

void Control::clipTorque()
{
    for(int8_t i = 0; i < 7; ++i)
    {
        torque_[i] = std::clamp(torque_[i], -torque_lim_[i], torque_lim_[i]);
    }
}

void Control::calcAllTerms()
{
    // 1. Матрица инерции M(q)
    M_ = pinocchio::crba(model_, data_, q_);

    // 2. Матрица Кориолисовых сил C(q, dq)
    C_ = pinocchio::computeCoriolisMatrix(model_, data_, q_, dq_);

    // 3. Вектор гравитационных сил g(q)
    g_ = pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

bool Control::setPDParameters(std_jarray Kp, std_jarray Kd)
{
    if ((Kp.size() == 7)&&(Kd.size() == 7))
    {
        Kp_ << Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5], Kp[6];
        Kd_ << Kd[0], Kd[1], Kd[2], Kd[3], Kd[4], Kd[5], Kd[6];
        return true;
    }
    return false;
}

std_jarray Control::calcTorquePD(const std_jarray& q, const std_jarray& q_d, const std_jarray& dq_d, const std_jarray& ddq_d)
{
    q_ << q[0], q[1], q[2], q[3], q[4], q[5], q[6];
    q_d_ << q_d[0], q_d[1], q_d[2], q_d[3], q_d[4], q_d[5], q_d[6];

    dq_d_ << dq_d[0], dq_d[1], dq_d[2], dq_d[3], dq_d[4], dq_d[5], dq_d[6];

    ddq_d_ << ddq_d[0], ddq_d[1], ddq_d[2], ddq_d[3], ddq_d[4], ddq_d[5], ddq_d[6];

    calcVel();

    calcAllTerms();

    u_ = Kp_.array()*(q_d_-q_).array() + Kd_.array()*(dq_d_-dq_).array() + ddq_d_.array();

    torque_ = M_*(u_) + C_*dq_ + g_;

    clipTorque();

    // ------------------------------------ Переделать + добавить клип
    std_jarray stdtorque;

    for (int8_t i = 0; i < torque_.size(); ++i) {
        stdtorque[i] = torque_[i];
    }

    return stdtorque;
}

