#pragma once

#include "Matrix4.hpp"

#include <vector>
#include <string>
#include <cassert>

class VertexArray {
public:
	VertexArray() {}
	~VertexArray() { clear(); }
	void clear() { texCoords.clear(); normals.clear(); vertices.clear(); faces.clear(); }

	VertexArray operator+(const VertexArray& v) const;

	void setNormals(float* newArr, int newArrCount);
	void setVertices(float* newArr, int newArrCount);
	void setFaces(int* newArr, int newArrCount);
	void setTexCoordSets(int newTCSets) { texCoords.clear(); texCoords.resize(newTCSets); }
	void setTexCoords(int set, float* newArr, int newArrCount);

	void add(float* newNorm, int newNormCount, float* newVert, int newVertCount, int* newFace, int newFaceCount,
			 float* newTC, int newTCCount);

	void getNormals(const float*& outputArrPtr, int& outputArrNum) const;
	void getVertices(const float*& outputArrPtr, int& outputArrNum) const;
	void getFaces(const int*& outputArrPtr, int& outputArrNum) const;
	int getTexCoordSets() const { return texCoords.size(); }
	void getTexCoords(int set, const float*& outputArrPtr, int& outputArrNum) const;

	int getNumFaces() const { return faces.size(); }
	// Array bounds are not checked for the following
	std::vector<float> getVertex(int faceNumber, int vertexNumber) {
		std::vector<float> v3(3);
		for (int i = 0; i < 3; ++i) v3[i] = vertices[faces[faceNumber*3+vertexNumber]*3+i];
		return v3;
	}
	std::vector<float> getNormal(int faceNumber, int vertexNumber) {
		std::vector<float> n3(3);
		for (int i = 0; i < 3; ++i) n3[i] = normals[faces[faceNumber*3+vertexNumber]*3+i];
		return n3;
	}
	std::vector<float> getTextureCoordinate(int faceNumber, int vertexNumber, int tcSet) {
		std::vector<float> t2(2);
		for (int i = 0; i < 2; ++i) t2[i] = texCoords[tcSet][faces[faceNumber*3+vertexNumber]*2+i];
		return t2;
	}

	void setToBillboard(float x1, float y1, float x2, float y2);
	void setBox4(float* ftx, float* fty);
	void setTo2DButton(float x, float y, float w, float h, float sideW, bool flip = false);
	void setTo2DBox(float x, float y, float w, float h, float marginW, float marginH, float clipX = 1.f);
	void setTo2DQuad(float x1, float y1, float x2, float y2, float u1, float v1, float u2, float v2, float z);
	void setVertexData2DQuad(float x1, float y1, float x2, float y2, float u1, float v1, float u2, float v2,
							 float* vCorners, float* uvs, int* bFaces, int faceOffset = 0) const;

	struct TriFloat {
		TriFloat(float nx, float ny, float nz) : x(nx), y(ny), z(nz) { }
		TriFloat() : x(0), y(0), z(0) { }
		float x, y, z;
	};

	struct TwoFloat {
		TwoFloat(float nu, float nv) : u(nu), v(nv) { }
		TwoFloat() : u(0), v(0) { }
		float u,v;
	};

	struct VertexData {
		VertexData() { }
		VertexData(TriFloat nv, TriFloat nn, TwoFloat nt) : vertex(nv), normal(nn), texCoord(nt) { }
		TriFloat vertex;
		TriFloat normal;
		TwoFloat texCoord;
		bool operator<(const VertexData& other) const {
				 if (vertex.x   != other.vertex.x)   return vertex.x   < other.vertex.x;
			else if (vertex.y   != other.vertex.y)   return vertex.y   < other.vertex.y;
			else if (vertex.z   != other.vertex.z)   return vertex.z   < other.vertex.z;
			else if (normal.x   != other.normal.x)   return normal.x   < other.normal.x;
			else if (normal.y   != other.normal.y)   return normal.y   < other.normal.y;
			else if (normal.z   != other.normal.z)   return normal.z   < other.normal.z;
			else if (texCoord.u != other.texCoord.u) return texCoord.u < other.texCoord.u;
			else if (texCoord.v != other.texCoord.v) return texCoord.v < other.texCoord.v;
			else 									 return false;
		}
	};

	struct Face {
		Face(VertexData nv1, VertexData nv2, VertexData nv3) : v1(nv1), v2(nv2), v3(nv3) { }
		VertexData v1, v2, v3;
		const VertexData& getVertexData(unsigned int index) const {
			assert(index < 3);

				 if (index == 2) return v3;
			else if (index == 1) return v2;
			else 				 return v1;
		}
	};

	void buildFromFaces(const std::vector<Face>& newFaces);
	void transform(const Matrix4<float>& m);
	void translate(float x, float y, float z);
	void rotate(float a, float x, float y, float z);

private:
	std::vector<std::vector<float> > texCoords;
	std::vector<float> normals;
	std::vector<float> vertices;
	std::vector<int> faces;
};
