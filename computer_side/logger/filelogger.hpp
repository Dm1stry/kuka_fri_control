#ifndef FILE_LOGGER_HPP
#define FILE_LOGGER_HPP

#include <fstream>
#include <string>
#include <map>

#include "ilogger.hpp"

namespace LOGGER
{

class FileLogger : public ILogger 
{
public:
	FileLogger(std::string name = "log");
	void setFileName(const std::string& name);
	virtual void log(const std::string& message) override;
	~FileLogger();
private:
	std::ofstream logger_stream_;
}; // class FileLogger

}; // namespace LOGGER

#endif // FILE_LOGGER_HPP