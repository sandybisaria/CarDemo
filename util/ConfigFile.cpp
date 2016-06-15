#include "ConfigFile.hpp"

#include <fstream>
#include <sys/stat.h>

ConfigFile::ConfigFile() {

}

ConfigFile::~ConfigFile() {

}

bool ConfigFile::load(std::string filename) {
	struct stat sb;
	if (stat(filename.c_str(), &sb) != 0 || !S_ISREG(sb.st_mode)) {
		return false; //TODO Error if directory not found
	}
	this->filename = filename;

	std::ifstream f;
	std::string curSection = "";
	const int MAX_CHAR = 1024;
	char trashChar[MAX_CHAR];

	while (f && !f.eof()) {
		f.getline(trashChar, MAX_CHAR, '\n');
		processLine(curSection, trashChar);
	}

	return true;
}

void ConfigFile::processLine(std::string& curSection, std::string lineStr) {
	lineStr = trim(lineStr);
	lineStr = strip(lineStr, '\r');
	lineStr = strip(lineStr, '\n');

	// Remove comments
	std::string::size_type commentPos = lineStr.find("#", 0);
	if (commentPos < lineStr.length()) {
		lineStr = lineStr.substr(0, commentPos);
	}

	lineStr = trim(lineStr);

	// Only continue if not a blank line or comment-only line
	if (lineStr.length() > 0) {
		if (lineStr.find("=", 0) < lineStr.length()) {
			// Find the name part
			std::string::size_type equalPos = lineStr.find("=", 0);
			std::string name = lineStr.substr(0, equalPos);
			equalPos++;
			std::string val = lineStr.substr(equalPos, lineStr.length() - equalPos);
			name = trim(name);
			val = trim(val);

			// Only continue if valid
			if (name.length() > 0 && val.length() > 0) {
				CONFIGVARIABLE newVar;
				newVar.section = curSection;
				newVar.name = name;
				newVar.Set(val);

				std::string paramName = name;
				if (!curSection.empty())
					paramName = curSection + "." + paramName;

				add(paramName, newVar);
			}
		}
		else {
			// Section header
			lineStr = strip(lineStr, '[');
			lineStr = strip(lineStr, ']');
			lineStr = trim(lineStr);
			curSection = lineStr;
		}
	}
}

std::string ConfigFile::trim(std::string s) {
	if (s.find_last_not_of(" \t") != std::string::npos) {
		s = s.erase(s.find_last_not_of(" \t") + 1);
	}
	return s.erase(0, s.find_first_not_of(" \t"));
}

std::string ConfigFile::strip(std::string s, char strip) {
	std::string::size_type pos = 0;
	std::string res = "";

	int length = s.length();
	while (pos < length) {
		if (s.c_str()[pos] == strip)
			break;
		pos++;
	}

	if (pos > 0) {
		res = s.substr(0, pos);
	}

	if (pos + 1 < length) {
		res = res + s.substr(pos+1, (length-pos)-1);
	}

	return res;
}

void ConfigFile::add(std::string& paramName, CONFIGVARIABLE& newVar) {
//	variables.Set(paramName, newVar);
}
