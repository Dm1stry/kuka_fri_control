#include "trajectory.hpp"

using namespace trajectory;

Trajectory::Trajectory(const Eigen::Array<double,7,1> &first_thetta)
{
    points_.push_back(first_thetta);

    virtual_thetta_ = first_thetta;

    eps_ << e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180, e*M_PI/180;
}

// =======================================================================

bool Trajectory::push(const Eigen::Array<double,7,1> &thetta)
{
    points_.push_back(thetta);
    return true;
}

bool Trajectory::pop(Eigen::Array<double,7,1> &thetta)
{
    if (points_.size() == 0)
    {
        return false;
    }
    thetta = points_.front();
    points_.pop_front();

    return true;
}

size_t Trajectory::size()
{
    return points_.size();
}

// =======================================================================

Eigen::Array<double,N_JOINTS,1> Trajectory::calcTransferedPoint()
{
    if (done_)
    {
        pop(next_thetta_);
        done_ = false;
    }

    virtual_thetta_ = virtual_thetta_ + getDelta(next_thetta_, virtual_thetta_, eps_);

    done_ = trajectory::eigenArrayEqual(next_thetta_, virtual_thetta_, eps_);

    return virtual_thetta_;
} 

void Trajectory::synchPosition(const Eigen::Array<double,N_JOINTS,1> &measured_thetta)
{
    virtual_thetta_ = measured_thetta;
}

// =======================================================================

Eigen::Array<double,N_JOINTS,1> Trajectory::getDelta(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta, const Eigen::Array<double,N_JOINTS,1> &eps)
{
    Eigen::Array<double,7,1> delta = next_thetta - current_thetta;

    for(int i = 0; i < N_JOINTS; ++i)
    {
        if(std::abs(delta[i]) < eps[i]) 
        {
            delta[i] = 0.;
        }
    }

    Eigen::Array<double,7,1> vel = v*Eigen::sign(delta);

    // std::cout << vel.transpose() << std::endl;

    return vel;
}

bool Trajectory::getDone()
{
    return done_;
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

bool trajectory::eigenArrayDiff(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &diff)
{
    for(int i = 0; i < N_JOINTS; ++i)
    {
        if(std::abs(arr1[i]-arr2[i]) > diff[i]) 
        {
            return true;
        }
    }
    return false;
}

void trajectory::setConnection()
{
    const char* name = "/my_shm";

    int shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return;
    }

    ftruncate(shm_fd, sizeof(bool));

    bool* ptr = (bool*)mmap(0, sizeof(bool), PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (ptr == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // const char* message = "Привет от первого процесса!";
    // std::memcpy(ptr, message, strlen(message) + 1);
    *ptr = true;

    munmap(ptr, sizeof(bool));
    close(shm_fd);
}