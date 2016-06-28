#include "ConfigFile.hpp"

#include <cstdlib>
#include <fstream>
#include <sys/stat.h>

// -----------------------------------------------------------------------------
// ConfigVariable methods
ConfigVariable::ConfigVariable()
	: mSection(""), mName("") {
	initVals();
}

ConfigVariable::ConfigVariable(std::string section, std::string name)
	: mSection(section), mName(name) {
	initVals();
}

void ConfigVariable::initVals() {
	val_s = "";
	val_i = 0;
	val_f = 0.f;
	val_b = false;
	for (int i = 0; i < V_SIZE; i++) {
		val_v[i] = 0.f;
	}
}

ConfigVariable::~ConfigVariable() {

}

void ConfigVariable::set(std::string newVal) {
	// Parses string to determine appropriate data type
	newVal = ConfigFile::trim(newVal);

	val_i = atoi(newVal.c_str());
	val_f = atof(newVal.c_str());
	val_s = newVal;

	val_b = false;
	if (val_i == 0)		val_b = false;
	if (val_i == 1)		val_b = true;

	if (ConfigFile::toLower(newVal) == "on")	val_b = true;
	if (ConfigFile::toLower(newVal) == "off")	val_b = false;
	if (ConfigFile::toLower(newVal) == "true")	val_b = true;
	if (ConfigFile::toLower(newVal) == "false")	val_b = false;

	// now process as vector information
	int pos = 0;
	int arraypos = 0;
	std::string::size_type nextpos = newVal.find(",", pos);
	std::string frag;

	while (nextpos < newVal.length() && arraypos < 3)
	{
		frag = newVal.substr(pos, nextpos - pos);
		val_v[arraypos] = atof(frag.c_str());

		pos = nextpos+1;
		++arraypos;
		nextpos = newVal.find(",", pos);
	}

	// don't forget the very last one
	if (arraypos < 3)
	{
		frag = newVal.substr(pos, newVal.length() - pos);
		val_v[arraypos] = atof(frag.c_str());
	}
}

// -----------------------------------------------------------------------------
// ConfigFile methods
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
	f.open(filename.c_str());
	if (!f) return false;

	return load(f);
}

bool ConfigFile::load(std::istream& f) {
	std::string curSection = "";
	const int MAX_CHAR = 1024;
	char trashChar[MAX_CHAR];

	while (f && !f.eof()) {
		f.getline(trashChar, MAX_CHAR, '\n');
		processLine(curSection, trashChar);
	}

	return true;
}

bool ConfigFile::getParam(std::string param, std::string& outVar) const {
	const ConfigVariable* v = getVariable(param);
	if (!v)
		return false;

	outVar = v->val_s;
	return true;
}

bool ConfigFile::getParam(std::string param, int& outVar) const {
	const ConfigVariable* v = getVariable(param);
	if (!v)
		return false;

	outVar = v->val_i;
	return true;
}

bool ConfigFile::getParam(std::string param, float& outVar) const {
	const ConfigVariable* v = getVariable(param);
	if (!v)
		return false;

	outVar = v->val_f;
	return true;
}

bool ConfigFile::getParam(std::string param, float* outVar) const {
	const ConfigVariable* v = getVariable(param);
	if (!v)
		return false;

	for (int i = 0; i < ConfigVariable::V_SIZE; ++i) {
		outVar[i] = v->val_v[i];
	}
	return true;
}

bool ConfigFile::getParam(std::string param, bool& outVar) const {
	const ConfigVariable* v = getVariable(param);
	if (!v)
		return false;

	outVar = v->val_b;
	return true;
}

void ConfigFile::getPoints(const std::string& sectionName, const std::string& paramPrefix,
			   	   	   	   std::vector<std::pair<double, double> >& outputPts) {
	std::list<std::string> params;
	getParamList(params, sectionName);
	for (std::list<std::string>::iterator i = params.begin(); i != params.end(); ++i) {
		if (i->find(paramPrefix) == 0) {
			float point[3] = {0, 0, 0};
			if (getParam(sectionName + "." + *i, point)) {
				outputPts.push_back(std::make_pair(point[0], point[1]));
			}
		}
	}
}

void ConfigFile::getParamList(std::list<std::string>& paramListOutput, std::string section) const {
	bool all = (section == ""); // If empty, then search everything

	paramListOutput.clear();
	std::map<std::string, bool> tempList;
	for (bucketed_hashmap<std::string, ConfigVariable>::const_iterator i = variables.begin();
		 i != variables.end(); i++) {
		if (all)
			tempList[i->mSection + "." + i->mName] = true;
		else if (i->mSection == section)
			tempList[i->mName] = true;
	}

	for (std::map<std::string, bool>::iterator i = tempList.begin(); i != tempList.end(); i++) {
		paramListOutput.push_back(i->first);
	}
}

void ConfigFile::getSectionList(std::list<std::string>& sectionListOutput) const {
	sectionListOutput.clear();

	std::map<std::string, bool> tempList;
	for(bucketed_hashmap<std::string, ConfigVariable>::const_iterator i = variables.begin(); i != variables.end(); i++) {
		tempList[i->mSection] = true;
	}
	for(std::map<std::string, bool>::iterator i = tempList.begin(); i != tempList.end(); i++) {
		sectionListOutput.push_back(i->first);
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

std::string ConfigFile::toLower(std::string s) {
	char tc[2];
	tc[1] = '\0';
	std::string res = "";

	std::string::size_type pos = 0;
	while (pos < s.length()) {
		// If an uppercase char
		if (s.c_str()[pos] <= 90 && s.c_str()[pos] >= 65) {
			tc[0] = s.c_str()[pos] + 32;
			res = res + tc;
		} else {
			res = res + s.substr(pos, 1);
		}

		pos++;
	}

	return res;
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
				ConfigVariable newVar(curSection, name);
				newVar.set(val);

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

void ConfigFile::add(std::string& paramName, ConfigVariable& newVar) {
	variables.Set(paramName, newVar);
}

const ConfigVariable* ConfigFile::getVariable(std::string param) const {
	std::string::size_type ppos;
	ppos = param.find(".", 0);
	if (ppos < param.length()) {
		if (param.substr(0, ppos).empty())
		{
			++ppos;
			param = param.substr(ppos, param.length() - ppos);
		}
	}

	const ConfigVariable* v = variables.Get(param);
	return v;
}

// -----------------------------------------------------------------------------
// Testing
#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(ConfigFile, ConfigFileFunctions) {
	std::stringstream inStream;
	inStream << "\n#comment on the FIRST LINE??\n\n"
				"variable outside of=a section\n\n"
				"test section numero UNO\n"
				"look at me = 23.4\n\n"
				"i'm so great=   BANANA\n"
				"#break!\n\n"
				"[ section    dos??]\n"
				"why won't you = breeeak #trying to break it\n\n"
				"what about ] # this malformed thing???\n"
				"nope works = fine.\n"
				"even vectors = 2.1,0.9,GAMMA\n"
				"this is a duplicate = 0\n"
				"this is a duplicate = 1\n"
				"random = intermediary\n"
				"this is a duplicate = 2\n";

	ConfigFile testConfig; testConfig.load(inStream);

	std::string tStr = "notfound";
	EXPECT_TRUE(testConfig.getParam("variable outside of", tStr));
	EXPECT_EQ(tStr, "a section");

	tStr = "notfound";
	EXPECT_TRUE(testConfig.getParam(".variable outside of", tStr));
	EXPECT_EQ(tStr, "a section");

	tStr = "notfound";
	EXPECT_TRUE(testConfig.getParam("section    dos??.why won't you", tStr));
	EXPECT_EQ(tStr, "breeeak");

	tStr = "notfound";
	EXPECT_TRUE(testConfig.getParam("variable outside of", tStr));
	EXPECT_EQ(tStr, "a section");

	tStr = "notfound";
	EXPECT_TRUE(testConfig.getParam(".variable outside of", tStr));
	EXPECT_EQ(tStr, "a section");

	tStr = "notfound";
	EXPECT_FALSE(testConfig.getParam("nosection.novariable", tStr));
	EXPECT_EQ(tStr, "notfound");

	tStr = "notfound";
	float vec[3];
	EXPECT_TRUE(testConfig.getParam("what about.even vectors", vec));
	EXPECT_EQ(vec[0], 2.1f);
	EXPECT_EQ(vec[1], 0.9f);
	EXPECT_EQ(vec[2], 0.f);

	std::list<std::string> list1;
	testConfig.getSectionList(list1);
	list1.sort();

	std::list<std::string>::iterator i = list1.begin();
	EXPECT_EQ(*i, ""); ++i;
	EXPECT_EQ(*i, "section    dos??"); ++i;
	EXPECT_EQ(*i, "test section numero UNO"); ++i;
	EXPECT_EQ(*i, "what about"); ++i;
	EXPECT_TRUE(i == list1.end());

	std::list<std::string> list2;
	testConfig.getParamList(list2, "test section numero UNO");
	list2.sort();

	std::list<std::string>::iterator j = list2.begin();
	EXPECT_EQ(*j, "i'm so great"); ++j;
	EXPECT_EQ(*j, "look at me"); ++j;
	EXPECT_TRUE(j == list2.end());
}

#endif
