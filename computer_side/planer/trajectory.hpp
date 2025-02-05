#ifndef TRAJECTORY
#define TRAJECTORY

#include <Eigen/Dense>
#include <list>

namespace trajectory
{
    const int N_JOINTS = 7;
    const double delta_thetta = 1*M_PI/180;
    const int points_per_delta = 5;


    class Trajectory
    {
        private:
            std::list<Eigen::Array<double,N_JOINTS,1>> points_;
            bool done_ = false;

            int checkPoints(const Eigen::Array<double,N_JOINTS,1> &delta);

        public:
            Trajectory(const Eigen::Array<double,N_JOINTS,1> &first_thetta);

            bool push(const Eigen::Array<double,N_JOINTS,1> &thetta);
            bool pop(Eigen::Array<double,N_JOINTS,1> &thetta);

            size_t size();

    };

    bool eigenArrayEqual(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &eps);

}

#endif