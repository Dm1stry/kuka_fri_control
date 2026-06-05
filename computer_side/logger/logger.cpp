#include "logger.hpp"

using namespace logger;

FileLogger::FileLogger(std::string file_name):
file_name_(file_name)
{
    
}

FileLogger::~FileLogger()
{
	stop();
}

void FileLogger::start()
{	
    // ofs.open(name, std::ofstream::out | std::ios_base::app);
    ofs_.open(file_name_, std::ofstream::out);

    if (!ofs_.is_open()) { 
        std::cout << "ERROR" << std::endl;
        return;
    } 
}

void FileLogger::stop()
{	
	if (ofs_.is_open()) 
	{
		closeFile();
	}
}

void FileLogger::closeFile() 
{
	ofs_.flush();
    ofs_.close(); 
}


bool FileLogger::setData(const Eigen::Array<double, 9, 1> &data)
{
	if (!ofs_.is_open())
	{
		return false;
	}

	ofs_ << data[0] << "," 
		 << data[1] << ","
		 << data[2] << ","
		 << data[3] << "," 
		 << data[4] << "," 
		 << data[5] << "," 
		 << data[6] << "," 
		 << data[7] << "," 
		 << data[8] << "\n";

	n_++;
	if (n_ == 100)
	{
		n_ = 0;
		ofs_.flush();
	}

	return true;
}
