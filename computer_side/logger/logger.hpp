#ifndef LOGGER
#define LOGGER

#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

#include <Eigen/Dense>

#include "../lockfree/lockfree.hpp"

namespace logger
{
    class FileLogger
    {
    private:
        std::string file_name_;         // file name 

	    std::ofstream ofs_;

        std::atomic<bool> logger_started_ = false;   // is receive started
        std::atomic<bool> data_ready_ = false;
        
        std::jthread write_;           // thread for receiving 

		// -----------------------------------------------------------------
        // ---------------------------------------------------- type of data
		// -----------------------------------------------------------------

        int n_ = 0;
        Eigen::Array<double, 9,1> data_;
        ring_buffer<Eigen::Array<double, 9,1>> write_buffer_;

		// -----------------------------------------------------------------------
        
        void closeFile();
        void run_write();

    public: 
        FileLogger(std::string file_name);
        ~FileLogger();

        void start();
        void stop();

        bool setData(Eigen::Array<double, 9,1> &data);
    };
};

#endif