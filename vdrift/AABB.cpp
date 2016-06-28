#include "AABB.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

void distribute(float frustum[][4]) {
	for (int i = 1; i < 6; i++)
		for (int n = 0; n < 4; n++)
			frustum[i][n] = frustum[0][n];
}

TEST(AABB, AABBFunctions) {
	AABB<float> box1, box2;
	MathVector<float, 3> c1, c2;

	c1.set(-1, -1, -1); c2.set(1, 1, 1);
	box1.setFromCorners(c1, c2);

	c1.set(-0.01, -0.01, 0); c2.set(0.01, 0.01, 2);
	box2.setFromCorners(c1, c2);

	EXPECT_TRUE(box1.intersects(box2));

	AABB <float> box3;
	c1.set(-0.01, -0.01, 2); c2.set(0.01, 0.01, 3);
	box3.setFromCorners(c1, c2);

	EXPECT_FALSE(box1.intersects(box3));

	MathVector<float, 3> orig, dir;
	orig.set(0, 0, 4); dir.set(0, 0, -1);

	EXPECT_TRUE(box1.intersects(AABB<float>::Ray(orig, dir, 4)));
	EXPECT_FALSE(box1.intersects(AABB<float>::Ray(orig, dir * -1, 4)));
	EXPECT_FALSE(box1.intersects(AABB<float>::Ray(orig, dir, 1)));

	{
		float plane[6][4];
		plane[0][0] = 0;
		plane[0][1] = 0;
		plane[0][2] = 1;
		plane[0][3] = 10;
		distribute(plane);
		EXPECT_TRUE(box1.intersects(AABB<float>::Frustum(plane)));
	}

	{
		float plane[6][4];
		plane[0][0] = 0;
		plane[0][1] = 0;
		plane[0][2] = 1;
		plane[0][3] = 0;
		distribute(plane);
		EXPECT_TRUE(box1.intersects(AABB<float>::Frustum(plane)));
	}

	{
		float plane[6][4];
		plane[0][0] = 0;
		plane[0][1] = 0;
		plane[0][2] = 1;
		plane[0][3] = -10;
		distribute(plane);
		EXPECT_FALSE(box1.intersects(AABB<float>::Frustum(plane)));
	}

	{
		float plane[6][4];
		plane[0][0] = -1;
		plane[0][1] = 0;
		plane[0][2] = 0;
		plane[0][3] = 10000;
		distribute(plane);
		EXPECT_TRUE(box1.intersects(AABB<float>::Frustum(plane)));
	}

	{
		float plane[6][4];
		plane[0][0] = 1;
		plane[0][1] = 0;
		plane[0][2] = 0;
		plane[0][3] = -119;
		distribute(plane);
		EXPECT_FALSE(box1.intersects(AABB<float>::Frustum(plane)));
	}
}

#endif
