#include "logger.hpp"

using namespace logger;

FileLogger::FileLogger(std::string file_name):
file_name_(file_name),
write_buffer_(8192)
{
    
}

FileLogger::~FileLogger()
{
	stop();
}

void FileLogger::start()
{	
	logger_started_ = true;

    // ofs.open(name, std::ofstream::out | std::ios_base::app);
    ofs_.open(file_name_, std::ofstream::out);

    if (!ofs_.is_open()) { 
        std::cout << "ERROR" << std::endl;
        return;
    } 

    write_ = std::jthread(&FileLogger::run_write, this);
}

void FileLogger::stop()
{	
	if (logger_started_) 
	{
		logger_started_ = false;
		if (write_.joinable()) {
			write_.join();
		}
		closeFile();
	}
}

void FileLogger::run_write()
{
    while (logger_started_)
	{
		if (data_ready_)
		{
			if (write_buffer_.pop(data_))
			{
				ofs_ << data_[0] << "," 
					 << data_[1] << ","
					 << data_[2] << ","
					 << data_[3] << "," 
					 << data_[4] << "," 
					 << data_[5] << "," 
					 << data_[6] << "," 
					 << data_[7] << "," 
					 << data_[8] << "\n";
				n_++;
				if (n_ == 100)
				{
					n_ = 0;
					ofs_.flush();
				}
			}
		}
	}
}

void FileLogger::closeFile() 
{
    ofs_.close(); 
}


bool FileLogger::setData(Eigen::Array<double, 9, 1> &data)
{

	data_ready_ = write_buffer_.push(data);

	return true;
}
