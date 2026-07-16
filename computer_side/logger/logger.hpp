#ifndef LOGGER
#define LOGGER

#include <fstream>
#include <iostream>

#include <Eigen/Dense>

namespace logger
{
    class FileLogger
    {
    private:
        std::string file_name_;         // file name 

	    std::ofstream ofs_;

		// -----------------------------------------------------------------
        // ---------------------------------------------------- type of data
		// -----------------------------------------------------------------

        int n_ = 0;

		// -----------------------------------------------------------------------
        
        void closeFile();

    public: 
        FileLogger(std::string file_name);
        ~FileLogger();

        void start();
        void stop();

        bool setData(const Eigen::Array<double, 9,1> &data);
    };
};

#endif
