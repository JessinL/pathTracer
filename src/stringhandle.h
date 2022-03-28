#ifndef MONTE_CARLO_PATH_TRACER_STRINGHANDLE_H_
#define MONTE_CARLO_PATH_TRACER_STRINGHANDLE_H_

#include <iostream>
#include <vector>
#include <sstream>

#define BUFFER_LENGTH 1024

std::vector<std::string> seperate_string(std::string origin);

std::vector<double> string2Doubles(std::string origin);

#endif