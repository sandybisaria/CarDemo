#include "VertexArray.hpp"

VertexArray VertexArray::operator+(const VertexArray& v) const {
	VertexArray out;

	// Combine vectors
	out.normals.reserve(normals.size() + v.normals.size());
	out.normals.insert(out.normals.end(), normals.begin(), normals.end());
	out.normals.insert(out.normals.end(), v.normals.begin(), v.normals.end());

	out.vertices.reserve(vertices.size() + v.vertices.size());
	out.vertices.insert(out.vertices.end(), vertices.begin(), vertices.end());
	out.vertices.insert(out.vertices.end(), v.vertices.begin(), v.vertices.end());

	int idxOffset = vertices.size() / 3;
	out.faces.reserve(faces.size() + v.faces.size());
	out.faces.insert(out.faces.end(), faces.begin(), faces.end());
	for (int i = 0; i < v.faces.size(); i++) {
		out.faces.push_back(v.faces[i] + idxOffset);
	}

	int maxTCSets = getTexCoordSets();
	if (v.getTexCoordSets() > maxTCSets) maxTCSets = v.getTexCoordSets();
	int tcSets1 = getTexCoordSets(), tcSets2 = v.getTexCoordSets();
	for (int i = 0; i < maxTCSets; i++) {
		if (i >= tcSets1 && i < tcSets2)
			out.texCoords[i] = v.texCoords[i];
		else if (i < tcSets1 && i >= tcSets2)
			out.texCoords[i] = texCoords[i];
		else if (i < tcSets1 && i < tcSets2) {
			out.texCoords[i].reserve(texCoords[i].size() + v.texCoords[i].size());
			out.texCoords[i].insert(out.texCoords[i].end(), texCoords[i].begin(), texCoords[i].end());
			out.texCoords[i].insert(out.texCoords[i].end(), v.texCoords[i].begin(), v.texCoords[i].end());
		}
	}

	return out;
}
