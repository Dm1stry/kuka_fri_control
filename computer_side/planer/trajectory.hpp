#ifndef TRAJECTORY
#define TRAJECTORY

#include <Eigen/Dense>
#include <list>
#include <iostream>

#include <iostream>
#include <fcntl.h>      // shm_open
#include <sys/mman.h>   // mmap, PROT_*, MAP_*
#include <unistd.h>     // ftruncate, close

namespace trajectory
{
    const int N_JOINTS = 7;
    const double delta_thetta = 1*M_PI/180;
    const int points_per_delta = 5;


    class Trajectory
    {
        private:

            double time_tick_ = 0.005;
            double v = 0.002;

            const double e = 0.1;
            Eigen::Array<double,N_JOINTS,1> eps_;

            std::list<Eigen::Array<double,N_JOINTS,1>> points_;
            bool done_ = true;

            Eigen::Array<double,N_JOINTS,1> virtual_thetta_;
            Eigen::Array<double,N_JOINTS,1> next_thetta_;

        public:
            Trajectory(const Eigen::Array<double,N_JOINTS,1> &first_thetta);

            bool push(const Eigen::Array<double,N_JOINTS,1> &thetta);
            bool pop(Eigen::Array<double,N_JOINTS,1> &thetta);

            Eigen::Array<double,N_JOINTS,1> getDelta(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta, const Eigen::Array<double,N_JOINTS,1> &eps);

            void synchPosition(const Eigen::Array<double,N_JOINTS,1> &measured_thetta);
            Eigen::Array<double,N_JOINTS,1> calcTransferedPoint();

            bool getDone();

            size_t size();

    };

    bool eigenArrayEqual(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &eps);
    bool eigenArrayDiff(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &diff);

    void setConnection();
}

#endif