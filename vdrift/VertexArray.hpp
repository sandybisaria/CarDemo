#pragma once

#include <vector>
#include <string>

//FIXME Finish

class VertexArray {
public:
	VertexArray() {}
	~VertexArray() { clear(); }

	VertexArray operator+(const VertexArray& v) const;

	void clear() { texCoords.clear(); normals.clear(); vertices.clear(); faces.clear(); }

	int getTexCoordSets() const { return texCoords.size(); }

private:
	std::vector<std::vector<float> > texCoords;
	std::vector<float> normals;
	std::vector<float> vertices;
	std::vector<int> faces;
};
