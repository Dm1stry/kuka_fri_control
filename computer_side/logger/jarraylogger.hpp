#ifndef JARRAY_LOGGER_HPP
#define JARRAY_LOGGER_HPP

#include <array>
#include "filelogger.hpp"

namespace LOGGER
{

using jarray = std::array<double, 7>;

class JArrayLogger : public FileLogger
{
public:
	JArrayLogger(const std::string& filename);
	void log(const jarray& state);
};

}; // namespace LOGGER

#endif // JARRAY_LOGGER_HPP