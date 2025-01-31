#include "jarraylogger.hpp"

using namespace LOGGER;

JArrayLogger::JArrayLogger(const std::string& filename) : FileLogger(filename) {}

void JArrayLogger::log(const jarray& state)
{
	FileLogger::log(std::to_string(state[0]) + "\t" +
					std::to_string(state[1]) + "\t" +
					std::to_string(state[2]) + "\t" +
					std::to_string(state[3]) + "\t" +
					std::to_string(state[4]) + "\t" +
					std::to_string(state[5]) + "\t" +
					std::to_string(state[6]) + "\n");
}