#pragma once

#include "MathVector.hpp"
#include "MathPlane.hpp"

#include <vector>

template <typename T>
class AABB {
public:
	AABB() {}
	AABB(const AABB<T>& other) : pos(other.pos), center(other.center), size(other.size) {}

	const AABB<T>& operator=(const AABB<T>& other) {
		pos = other.pos;
		center = other.center;
		size = other.size;
		return *this;
	}

	const MathVector<T, 3>& getPosition() const { return pos; }
	const MathVector<T, 3>& getCenter() const {	return center; }
	const MathVector<T, 3>& getSize() const { return size; }

	void setFromCorners(const MathVector<T, 3>& c1, const MathVector<T, 3>& c2) {
		MathVector<T, 3> c1mod(c1);
		MathVector<T, 3> c2mod(c2);

		// Ensure c1 is smaller than c2
		if (c1[0] > c2[0]) {
			c1mod[0] = c2[0];
			c2mod[0] = c1[0];
		}
		if (c1[1] > c2[1]) {
			c1mod[1] = c2[1];
			c2mod[1] = c1[1];
		}
		if (c1[2] > c2[2]) {
			c1mod[2] = c2[2];
			c2mod[2] = c1[2];
		}

		pos = c1mod;
		size = c2mod - c1mod;
		center = pos + size * 0.5;
	}

	void combineWith(const AABB<T>& other) {
		const MathVector<T, 3>& otherPos(other.pos);

		MathVector<T, 3> min(pos), max(pos + size);
		if (otherPos[0] < min[0])
			min[0] = otherPos[0];
		if (otherPos[1] < min[1])
			min[1] = otherPos[1];
		if (otherPos[2] < min[2])
			min[2] = otherPos[2];

		const MathVector<T, 3>& otherMax(otherPos + other.size);
		if (otherMax[0] > max[0])
			max[0] = otherMax[0];
		if (otherMax[1] > max[1])
			max[1] = otherMax[1];
		if (otherMax[2] > max[2])
			max[2] = otherMax[2];

		setFromCorners(min, max);
	}

	class Ray {
	public:
		Ray(const MathVector<T, 3>& newOrig, const MathVector<T, 3>& newDir, T newSegLen)
			: orig(newOrig), dir(newDir), segLen(newSegLen) { }

		MathVector<T, 3> orig, dir;
		T segLen;
	};

	bool intersects(const Ray& ray) const {
		MathVector<T,3> segDir(ray.dir * (0.5f * ray.segLen));
		MathVector<T,3> segCenter(ray.orig + segDir);
		MathVector<T,3> diff(segCenter - center);

		MathVector<T,3> absSegDir(segDir);
		absSegDir.toAbsVal();
		MathVector<T,3> absDiff(diff);
		absDiff.toAbsVal();

		T f = size[0] + absSegDir[0];
		if (absDiff[0] > f)	return false;
		f = size[1] + absSegDir[1];
		if (absDiff[1] > f)	return false;
		f = size[2] + absSegDir[2];
		if (absDiff[2] > f)	return false;

		MathVector<T,3> cross(segDir.cross(diff));

		MathVector<T,3> absCross(cross);
		absCross.toAbsVal();

		f = size[1] * absSegDir[2] + size[2] * absSegDir[1];
		if (absCross[0] > f) return false;
		f = size[2] * absSegDir[0] + size[0] * absSegDir[2];
		if (absCross[1] > f) return false;
		f = size[0] * absSegDir[1] + size[1] * absSegDir[0];
		if (absCross[2] > f) return false;

		return true;
	}

	bool intersects(const AABB<T>& other) const {
		MathVector<T, 3> otherC1(other.pos);
		MathVector<T, 3> otherC2(otherC1 + other.size);

		MathVector<T, 3> c1(pos);
		MathVector<T, 3> c2(pos + size);

		// Check for non-collisions
		if (c1[0] > otherC2[0] || c2[0] < otherC1[0]) return false;
		if (c1[2] > otherC2[2] || c2[2] < otherC1[2]) return false;
		if (c1[1] > otherC2[1] || c2[1] < otherC1[1]) return false;

		return true;
	}

	struct Frustum {
		Frustum() : planes(6) {}
		Frustum(T cFrustum[][4]) : planes(6) { set(cFrustum); }

		void set(T cFrustum[][4]) {
			for (int i = 0; i < 6; i++)
				planes[i].set(cFrustum[i]);
		}

		std::vector<MathPlane<T> > planes;
	};

	bool intersects(const Frustum& frustum) const {
		float rd;
		const float bound = size.magnitude() * 0.5;

		for (int i = 0; i < 6; i++) {
			rd = frustum.planes[i][0] * center[0] +
				 frustum.planes[i][1] * center[1] +
				 frustum.planes[i][2] * center[2] +
				 frustum.planes[i][3];
			if (rd <= -bound)
				return false;
		}

		return true;
	}

private:
	MathVector<T, 3> pos; // Minimum corner (center - size/2)
	MathVector<T, 3> center; // Exact center
	MathVector<T, 3> size;
};
