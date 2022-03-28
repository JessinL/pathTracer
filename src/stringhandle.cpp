#include "stringhandle.h"

std::vector<std::string> seperate_string(std::string origin) {
	std::vector<std::string> result;
	std::stringstream ss(origin);
	while (ss >> origin) result.push_back(origin);
	return result;
}

std::vector<double> string2Doubles(std::string origin) {
	int begin = 0, end = 0;
	for (int i = 0; i < origin.size(); i++) {
		if ((origin[i] >= '0' && origin[i] <= '9') || (origin[i] == '-')) {
			begin = i;
			break;
		}
	}
	for (int i = begin; i < origin.size(); i++) {
		if ((origin[i] > '9' || origin[i] < '0') && (origin[i] != ' ') && (origin[i] != '.') && (origin[i] != ',') && (origin[i] != '-')) {
			end = i;
			break;
		}
	}
	origin = origin.substr(begin, end - begin);
	std::vector<std::string> words = seperate_string(origin);
	std::vector<double> result;
	for (int i = 0; i < words.size(); i++)
		result.push_back(std::stod(words[i]));
	return result;
}