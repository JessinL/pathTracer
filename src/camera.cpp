#include "camera.h"
#include "stringhandle.h"

int Camera::readXmlFile(const char* filename) {
	std::string stringfilename(filename);
	if (!(stringfilename.substr(stringfilename.size() - 3, 3) == "xml"))
		stringfilename.append(".xml");
	std::ifstream fp(stringfilename);
	if (!fp) {
		std::cout << "No such .xml file - " << stringfilename << std::endl;
		return 1;
	}
	else
		std::cout << "Successfully open file -" << stringfilename << std::endl;
	char buffer[BUFFER_LENGTH];
	std::string tempstr;
	// the first line: version, encoding
	fp.getline(buffer, BUFFER_LENGTH);
	// the second line: camera type
	fp.getline(buffer, BUFFER_LENGTH);
	// the third line: eye
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	std::vector<double> words = string2Doubles(tempstr);
	eye.x = words[0]; eye.y = words[1]; eye.z = words[2];
	// the fourth line: lookat
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	words = string2Doubles(tempstr);
	lookat.x = words[0]; lookat.y = words[1]; lookat.z = words[2];
	// the fifth line: up
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	words = string2Doubles(tempstr);
	up.x = words[0];  up.y = words[1]; up.z = words[2];
	// the sixth line: fovy
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	words = string2Doubles(tempstr);
	fovy = words[0];
	// the seventh line: width
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	words = string2Doubles(tempstr);
	width = std::round(words[0]);
	// the eighth line: height
	fp.getline(buffer, BUFFER_LENGTH);
	tempstr = buffer;
	words = string2Doubles(tempstr);
	height = std::round(words[0]);

	std::cout << "Read .xml file end." << std::endl;

	return 0;
}