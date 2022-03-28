#include "light.h"
#include "stringhandle.h"

int Light::readXmlFile(const char* filename) {
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
	for(int i = 0; i < 9; i++)
		fp.getline(buffer, BUFFER_LENGTH);
	while (!fp.eof()) {
		fp.getline(buffer, BUFFER_LENGTH);
		tempstr = buffer;
		int begin, end;
		for (int i = 0; i < tempstr.size(); i++) {
			if (tempstr[i] == '"') {
				begin = i;
				break;
			}
		}
		for (int i = begin + 1; i < tempstr.size(); i++) {
			if (tempstr[i] == '"') {
				end = i;
				break;
			}
		}
		OneLight light;
		light.name = tempstr.substr(begin + 1, end - begin - 1);
		tempstr = tempstr.substr(end + 1, tempstr.size() - 1);
		std::vector<double> words = string2Doubles(tempstr);
		light.radiance.x = words[0]; light.radiance.y = words[1]; light.radiance.z = words[2];
		lights.push_back(light);
	}

	return 0;
}