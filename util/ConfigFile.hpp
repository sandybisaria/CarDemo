#pragma once

#include <string>

class ConfigVariable {
public:
	ConfigVariable();
	ConfigVariable(std::string section, std::string name);
	~ConfigVariable();

	void set(std::string newVal);

private:
	std::string mSection;
	std::string mName;

	static const int V_SIZE = 3;
	void initVals();

	// Possible value types
	std::string val_s;
	int val_i;
	float val_f;
	float val_v[V_SIZE];
	bool val_b;
};

class ConfigFile {
public:
	ConfigFile();
	~ConfigFile();

	bool load(std::string filename);

private:
	void processLine(std::string& curSection, std::string lineStr);

	static std::string trim(std::string s);
	static std::string strip(std::string s, char strip);

	std::string filename;
};
