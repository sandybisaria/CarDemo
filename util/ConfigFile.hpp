#pragma once

#include <string>

class ConfigFile {
public:
	ConfigFile();
	~ConfigFile();

	bool load(std::string filename);

private:
	void processLine(std::string& curSection, std::string lineStr);
	std::string trim(std::string s);
	std::string strip(std::string s, char strip);
	void add(std::string& paramName, CONFIGVARIABLE& newVar);

	std::string filename;
};
