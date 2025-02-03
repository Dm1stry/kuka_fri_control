#ifndef ILOGGER_HPP
#define ILOGGER_HPP

#include <string>
namespace LOGGER
{

class ILogger
{
public:
	virtual void log(const std::string& message) = 0;
}; // class ILogger

}; // namespace LOGGER

#endif // ILOGGER_HPP