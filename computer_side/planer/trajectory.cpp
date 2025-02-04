#include "trajectory.hpp"

using namespace trajectory;

Trajectory::Trajectory(const Eigen::Array<double,7,1> &first_thetta)
{
    points_.push_back(first_thetta);
}

// =======================================================================

bool Trajectory::push(const Eigen::Array<double,7,1> &thetta)
{
    Eigen::Array<double,N_JOINTS,1> previous_thetta = points_.back();
    Eigen::Array<double,N_JOINTS,1> delta = thetta - previous_thetta;

    int n = checkPoints(delta)*points_per_delta;

    if (n > 0)
    {
        for (int i = 1; i <= n; ++i)
        {
            points_.push_back(previous_thetta + delta*i/n);
        }
    }
    else
    {
        points_.push_back(thetta);
    }

    if (done_ == true)
    {
        points_.pop_front();
        done_ = false;
    }
    return true;
}

bool Trajectory::pop(Eigen::Array<double,7,1> &thetta)
{
    thetta = points_.front();

    if (points_.size() == 1)
    {
        done_ = true;
    }
    else
    {
        done_ = false;
        points_.pop_front();
    }
    return true;
}

size_t Trajectory::size()
{
    return points_.size();
}

// =======================================================================

int Trajectory::checkPoints(const Eigen::Array<double,N_JOINTS,1> &delta)
{
    int n_points_max = 0;
    int n_points = 0;

    for(int i = 0; i < N_JOINTS; ++i)
    {
        if((delta[i] >= delta_thetta)||(delta[i] <= -delta_thetta))
        {
            n_points = (int)(delta[i]/delta_thetta);
            if (n_points > n_points_max) n_points_max = n_points;
        }
    }
    return n_points_max;
}

// =======================================================================

bool trajectory::eigenArrayEqual(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &eps)
{
    for(int i = 0; i < N_JOINTS; ++i)
    {
        if(std::abs(arr1[i]-arr2[i]) > eps[i]) 
        {
            return false;
        }
    }
    return true;
}