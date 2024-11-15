#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>

using namespace KUKA_CONTROL;

int main(int argc, char **argv)
{   
    KukaFRIController kuka;
    double _amplRad = 0.04;
    double _phi = 0.0;
    double _offset = 0.0;
    double _filterCoeff = 0.99;
    double sample_time = 0.001;
    double _freqHz = 0.25;
    double _stepWidth = 2 *M_PI *_freqHz * sample_time;
    
    kuka.start();
    while(true)
    {
        double newOffset = _amplRad * sin(_phi);
        _offset = _offset * _filterCoeff + newOffset * (1.0 - _filterCoeff);
        _phi += _stepWidth;
        if (_phi >= 2 * M_PI)
            _phi -= 2 * M_PI;
        kuka.moveJointPositionAt({0, 0, 0, _offset, 0, 0, 0});
        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    return 0;
}
