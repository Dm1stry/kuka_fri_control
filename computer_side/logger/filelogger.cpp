#include "filelogger.hpp"

using namespace LOGGER;

FileLogger::FileLogger(std::string name /* = "log" */)
{
	setFileName(name);
}

void FileLogger::setFileName(const std::string& name)
{
	logger_stream_.open(name + ".txt", std::ios_base::out);
}

void FileLogger::log(const std::string& message)
{
	logger_stream_ << message << std::endl;
}

FileLogger::~FileLogger()
{
	logger_stream_.close();
}