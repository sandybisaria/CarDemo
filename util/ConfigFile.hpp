#pragma once

#include "bucketed_hashmap.hpp"

#include <iostream>
#include <list>
#include <map>
#include <string>
#include <vector>

// Stuntrally's CONFIGVARIABLE class
class ConfigVariable {
	friend class ConfigFile;

public:
	ConfigVariable();
	ConfigVariable(std::string section, std::string name);
	~ConfigVariable() { }

	void set(std::string newVal);

	static const int V_SIZE = 3;

private:
	std::string mSection;
	std::string mName;

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
	ConfigFile() { }
	~ConfigFile() { }

	bool load(std::string filename);
	bool load(std::istream& f);

//---- Return true if param was found
	bool getParam(std::string param, std::string& outVar) const;
	bool getParam(std::string param, int& outVar) const;
	bool getParam(std::string param, float& outVar) const;
	bool getParam(std::string param, float* outVar) const; // For float[]
	bool getParam(std::string param, bool& outVar) const;

	void getPoints(const std::string& sectionName, const std::string& paramPrefix,
				   std::vector<std::pair<double, double> >& outputPts);

	void getParamList(std::list<std::string>& paramListOutput, std::string section) const;

	void getSectionList(std::list<std::string>& sectionListOutput) const;

	// May want to use C++ library functions rather than our own
	static std::string trim(std::string s);
	static std::string strip(std::string s, char strip);
	static std::string toLower(std::string s);

private:
	void processLine(std::string& curSection, std::string lineStr);
	void add(std::string& paramName, ConfigVariable& newVar);

	const ConfigVariable* getVariable(std::string param) const;

	std::string filename;
	bucketed_hashmap<std::string, ConfigVariable> variables;
};
