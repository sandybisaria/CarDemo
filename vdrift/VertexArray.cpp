#include "VertexArray.hpp"

#include "MathVector.hpp"
#include "Quaternion.hpp"

#include <algorithm>
#include <map>

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

void VertexArray::setNormals(float* newArr, int newArrCount) {
	// Tried to assign values that aren't in sets of 3
	assert(newArrCount % 3 == 0);

	if (newArrCount != normals.size()) normals.resize(newArrCount);

	float *myArr = &(normals[0]);
	for (size_t i = 0; i < newArrCount; ++i) myArr[i] = newArr[i];
}

void VertexArray::setVertices(float* newArr, int newArrCount) {
	// Tried to assign values that aren't in sets of 3
	assert(newArrCount % 3 == 0);

	if (newArrCount != vertices.size()) vertices.resize(newArrCount);

	float *myArr = &(vertices[0]);
	for (size_t i = 0; i < newArrCount; ++i) myArr[i] = newArr[i];
}

void VertexArray::setFaces(int* newArr, int newArrCount) {
	// Tried to assign values that aren't in sets of 3
	assert(newArrCount % 3 == 0);

	if (newArrCount != faces.size()) faces.resize(newArrCount);

	int *myArr = &(faces[0]);
	for (size_t i = 0; i < newArrCount; ++i) myArr[i] = newArr[i];
}

void VertexArray::setTexCoords(int set, float* newArr, int newArrCount) {
	assert(set < texCoords.size());

	// Tried to assign values that aren't in sets of 2
	assert(newArrCount % 2 == 0);

	if (newArrCount != texCoords[set].size()) texCoords[set].resize(newArrCount);

	float *myArr = &(texCoords[set][0]);
	for (size_t i = 0; i < newArrCount; ++i) myArr[i] = newArr[i];
}

void VertexArray::getNormals(const float*& outputArrPtr, int& outputArrNum) const {
	outputArrNum = normals.size();
	outputArrPtr = normals.empty() ? NULL : &normals[0];
}

void VertexArray::getVertices(const float*& outputArrPtr, int& outputArrNum) const {
	outputArrNum = vertices.size();
	outputArrPtr = vertices.empty() ? NULL : &vertices[0];
}

void VertexArray::getFaces(const int*& outputArrPtr, int& outputArrNum) const {
	outputArrNum = faces.size();
	if (faces.size() < 1) outputArrPtr = NULL; //TODO Why not just faces.empty()?
	else outputArrPtr = &faces[0];
}

void VertexArray::getTexCoords(int set, const float*& outputArrPtr, int& outputArrNum) const{
	assert(set < texCoords.size());

	outputArrNum = texCoords[set].size();
	outputArrPtr = texCoords[set].empty() ? NULL : &texCoords[set][0];
}

void VertexArray::setToBillboard(float x1, float y1, float x2, float y2) {
	int bfaces[6];
	bfaces[0] = 0;
	bfaces[1] = 1;
	bfaces[2] = 2;
	bfaces[3] = 0;
	bfaces[4] = 2;
	bfaces[5] = 3;
	setFaces(bfaces, 6);

	float normals[12];
	for (int i = 0; i < 12; i += 3) {
		normals[i] = 0;
		normals[i+1] = 0;
		normals[i+2] = 1;
	}
	setNormals(normals, 12);

	float verts[12];
	verts[2] = verts[5] = verts[8] = verts[11] = 0.0;
	verts[0] = verts[9] = x1;
	verts[3] = verts[6] = x2;
	verts[1] = verts[4] = y1;
	verts[7] = verts[10] = y2;
	setVertices(verts, 12);

	float tc[8];
	tc[0] = tc[1] = tc[3] = tc[6] = 0.0;
	tc[2] = tc[4] = tc[5] = tc[7] = 1.0;
	setTexCoordSets(1);
	setTexCoords(0, tc, 8);
}

void VertexArray::setBox4(float* ftx, float* fty) {
	int bfaces[6];
	bfaces[0] = 0;	bfaces[1] = 1;	bfaces[2] = 2;
	bfaces[3] = 0;	bfaces[4] = 2;	bfaces[5] = 3;
	setFaces(bfaces, 6);

	float normals[12];
	for (int i = 0; i < 12; i += 3) {
		normals[i] = 0;
		normals[i+1] = 0;
		normals[i+2] = 1;
	}
	setNormals(normals, 12);

	float verts[12];
	verts[0] = ftx[0];	verts[1] = fty[0];	verts[2] = 0.0;
	verts[3] = ftx[1];	verts[4] = fty[1];	verts[5] = 0.0;
	verts[6] = ftx[2];	verts[7] = fty[2];	verts[8] = 0.0;
	verts[9] = ftx[3];	verts[10]= fty[3];	verts[11] = 0.0;
	setVertices(verts, 12);

	float tc[8];
	tc[0] = tc[1] = tc[3] = tc[6] = 0.0;
	tc[2] = tc[4] = tc[5] = tc[7] = 1.0;
	setTexCoordSets(1);
	setTexCoords(0, tc, 8);
}

void VertexArray::setTo2DButton(float x, float y, float w, float h, float sideW, bool flip) {
	float vcorners[12*3];
	float uvs[8*3];
	int bfaces[6*3];

	MathVector<float, 2> corner1;
	MathVector<float, 2> corner2;
	MathVector<float, 2> dim;
	dim.set(w,h);
	MathVector<float, 2> center;
	center.set(x,y);
	corner1 = center - (dim * 0.5);
	corner2 = center + (dim * 0.5);

	float x1 = corner1[0];
	float y1 = corner1[1];
	float x2 = corner2[0];
	float y2 = corner2[1];

	if (flip) {
		float y3 = y1;
		y1 = y2;
		y2 = y3;
	}

	// Left
	setVertexData2DQuad(x1-sideW,y1,x1,y2, 0,0,0.5,1, vcorners, uvs, bfaces);

	// Center
	setVertexData2DQuad(x1,y1,x2,y2, 0.5,0,0.5,1, &(vcorners[12]), &(uvs[8]), &(bfaces[6]), 4);

	// Right
	setVertexData2DQuad(x2,y1,x2+sideW,y2, 0.5,0,1,1, &(vcorners[12*2]), &(uvs[8*2]), &(bfaces[6*2]), 8);

	setFaces(bfaces, 6*3);
	setVertices(vcorners, 12*3);
	setTexCoordSets(1);
	setTexCoords(0, uvs, 8*3);
}

void VertexArray::setTo2DBox(float x, float y, float w, float h, float marginW, float marginH, float clipX) {
	const unsigned int quads = 9;
	float vcorners[12*quads];
	float uvs[8*quads];
	int bfaces[6*quads];

	MathVector<float, 2> corner1;
	MathVector<float, 2> corner2;
	MathVector<float, 2> dim;
	dim.set(w, h);
	MathVector<float, 2> center;
	center.set(x,y);
	corner1 = center - dim * 0.5;
	corner2 = center + dim * 0.5;
	MathVector<float, 2> margin;
	margin.set(marginW, marginH);

	float lxmax = std::max((corner1-margin)[0],std::min(clipX,corner1[0]));
	float cxmax = std::max(corner1[0],std::min(clipX,corner2[0]));
	float rxmax = std::max(corner2[0],std::min(clipX,(corner2+margin)[0]));
	float lumax = (lxmax-(corner1-margin)[0])/(corner1[0]-(corner1-margin)[0])*0.5;
	float rumax = (rxmax-corner2[0])/((corner2+margin)[0]-corner2[0])*0.5+0.5;

	// Upper left
	setVertexData2DQuad((corner1-margin)[0],(corner1-margin)[1],lxmax,corner1[1],
						0,0,lumax,0.5, vcorners,uvs,bfaces);

	// Upper center
	setVertexData2DQuad(corner1[0],(corner1-margin)[1],cxmax,corner1[1],
						0.5,0,0.5,0.5,
						&(vcorners[12*1]),&(uvs[8*1]),&(bfaces[6*1]),4*1);

	// Upper right
	setVertexData2DQuad(corner2[0],(corner1-margin)[1],rxmax,corner1[1],
						0.5,0,rumax,0.5,
						&(vcorners[12*2]),&(uvs[8*2]),&(bfaces[6*2]),4*2);

	// Center left
	setVertexData2DQuad((corner1-margin)[0],corner1[1],lxmax,corner2[1],
						0,0.5,lumax,0.5,
						&(vcorners[12*3]),&(uvs[8*3]),&(bfaces[6*3]),4*3);

	// Center center
	setVertexData2DQuad(corner1[0],corner1[1],cxmax,corner2[1],
			    0.5,0.5,0.5,0.5,
			    &(vcorners[12*4]),&(uvs[8*4]),&(bfaces[6*4]),4*4);

	// Center right
	setVertexData2DQuad(corner2[0],corner1[1],rxmax,corner2[1],
						0.5,0.5,rumax,0.5,
						&(vcorners[12*5]),&(uvs[8*5]),&(bfaces[6*5]),4*5);

	// Lower left
	setVertexData2DQuad((corner1-margin)[0],corner2[1],lxmax,(corner2+margin)[1],
						0,0.5,lumax,1,
						&(vcorners[12*6]),&(uvs[8*6]),&(bfaces[6*6]),4*6);

	// Lower center
	setVertexData2DQuad(corner1[0],corner2[1],cxmax,(corner2+margin)[1],
						0.5,0.5,0.5,1,
						&(vcorners[12*7]),&(uvs[8*7]),&(bfaces[6*7]),4*7);

	// Lower right
	setVertexData2DQuad(corner2[0],corner2[1],rxmax,(corner2+margin)[1],
						0.5,0.5,rumax,1,
						&(vcorners[12*8]),&(uvs[8*8]),&(bfaces[6*8]),4*8);

	setFaces(bfaces, 6*quads);
	setVertices(vcorners, 12*quads);
	setTexCoordSets(1);
	setTexCoords(0, uvs, 8*quads);
}

void VertexArray::setTo2DQuad(float x1, float y1, float x2, float y2, float u1, float v1, float u2, float v2, float z) {
	float vcorners[12];
	float uvs[8];
	int bfaces[6];
	setVertexData2DQuad(x1,y1,x2,y2,u1,v1,u2,v2, vcorners, uvs, bfaces);
	for (int i = 2; i < 12; i += 3) vcorners[i] = z;
	setFaces(bfaces, 6);
	setVertices(vcorners, 12);
	setTexCoordSets(1);
	setTexCoords(0, uvs, 8);
}

void VertexArray::setVertexData2DQuad(float x1, float y1, float x2, float y2, float u1, float v1, float u2, float v2,
		 	 	 	 	 	 	 	  float* vCorners, float* uvs, int* bFaces, int faceOffset) const {
	vCorners[0] = x1;
	vCorners[1] = y1;
	vCorners[2] = 0;
	vCorners[3] = x2;
	vCorners[4] = y1;
	vCorners[5] = 0;
	vCorners[6] = x2;
	vCorners[7] = y2;
	vCorners[8] = 0;
	vCorners[9] = x1;
	vCorners[10] = y2;
	vCorners[11] = 0;

	uvs[0] = u1;
	uvs[1] = v1;
	uvs[2] = u2;
	uvs[3] = v1;
	uvs[4] = u2;
	uvs[5] = v2;
	uvs[6] = u1;
	uvs[7] = v2;

	bFaces[0] = faceOffset+0;
	bFaces[1] = faceOffset+2;
	bFaces[2] = faceOffset+1;
	bFaces[3] = faceOffset+0;
	bFaces[4] = faceOffset+3;
	bFaces[5] = faceOffset+2;
}

void VertexArray::buildFromFaces(const std::vector<Face>& newFaces) {
	std::map<VertexData, int> indexMap;
	clear();
	texCoords.resize(1);

	// Loop through triangles
	for (std::vector<Face>::const_iterator i = newFaces.begin(); i != newFaces.end(); ++i) {
		// Loop through vertices in each triangle
		for (int v = 0; v < 3; ++v) {
			const VertexData& curvertdata = i->getVertexData(v);
			std::map<struct VertexData, int>::iterator result = indexMap.find(curvertdata);
			if (result == indexMap.end()) {
				// New vertex
				unsigned int newidx = indexMap.size();
				indexMap[curvertdata] = newidx;

				vertices.push_back(curvertdata.vertex.x);
				vertices.push_back(curvertdata.vertex.y);
				vertices.push_back(curvertdata.vertex.z);

				normals.push_back(curvertdata.normal.x);
				normals.push_back(curvertdata.normal.y);
				normals.push_back(curvertdata.normal.z);

				texCoords[0].push_back(curvertdata.texCoord.u);
				texCoords[0].push_back(curvertdata.texCoord.v);

				faces.push_back(newidx);
			}
			else // Non-unique vertex
				faces.push_back(result->second);
		}
	}

	assert(faces.size()/3 == newFaces.size());
	assert(vertices.size()/3 == normals.size()/3 && normals.size()/3 == texCoords[0].size()/2);
	assert(vertices.size()/3 <= faces.size());
}

void VertexArray::transform(const Matrix4<float>& m) {
	Matrix4<float> mat = m;

	// Rotate and translate vertices
	assert(vertices.size() % 3 == 0);
	for(unsigned int i = 0; i < vertices.size(); i += 3)
		mat.transformVectorOut(vertices[i], vertices[i+1], vertices[i+2]);

	// Rotate normals
	mat[12] = 0; mat[13] = 0; mat[14] = 0;
	assert(normals.size() % 3 == 0);
	for(unsigned int i = 0; i < normals.size(); i += 3)
		mat.transformVectorOut(normals[i], normals[i+1], normals[i+2]);
}

void VertexArray::translate(float x, float y, float z) {
	assert(vertices.size() % 3 == 0);
	for (std::vector<float>::iterator i = vertices.begin(); i != vertices.end(); i += 3) {
		float * vert = &*i;
		vert[0] += x;
		vert[1] += y;
		vert[2] += z;
    }
}

void VertexArray::rotate(float a, float x, float y, float z) {
	Quaternion<float> q;
	q.rotate(a,x,y,z);

	assert(vertices.size() % 3 == 0);
	for (std::vector<float>::iterator i = vertices.begin(); i != vertices.end(); i += 3) {
		float * vert = &*i;
		q.rotateVector(vert);
    }
}

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(VertexArray, VertexArrayFunctions) {
	EXPECT_TRUE(true);
}
#endif
