#pragma once

#include <assert.h>

class Model;
class TextureGL;
class TerrainSurface;

// Based on
class RoadObject {
public:
	RoadObject(Model* m, TextureGL* t, bool hasSurface)
		: model(m), texture(t), surface(hasSurface) {
		assert(model);
		assert(texture);
	}

	Model* getModel() const { return model; }

	bool hasSurface() const { return surface; }

private:
	Model* model;
	TextureGL* texture;
	bool surface;
};