#include "Quaternion.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

#include <cmath>
#define _USE_MATH_DEFINES

TEST(Quaternion, QuaternionFunctions) {
	Quaternion<float> test1;
	EXPECT_EQ(test1.x(), 0.0);
	EXPECT_EQ(test1.y(), 0.0);
	EXPECT_EQ(test1.z(), 0.0);
	EXPECT_EQ(test1.w(), 1.0);

	Quaternion<float> test2; test2.loadIdentity();
	EXPECT_EQ(test1,test2);

	test1.w() = 0.7;
	test1.normalize();
	EXPECT_EQ(test1.magnitude(), 1.0);

	float mat[16];
	test1.representAsMatrix4(mat);
	EXPECT_EQ(mat[0],  1.0);
	EXPECT_EQ(mat[1],  0.0);
	EXPECT_EQ(mat[2],  0.0);
	EXPECT_EQ(mat[3],  0.0);
	EXPECT_EQ(mat[4],  0.0);
	EXPECT_EQ(mat[5],  1.0);
	EXPECT_EQ(mat[6],  0.0);
	EXPECT_EQ(mat[7],  0.0);
	EXPECT_EQ(mat[8],  0.0);
	EXPECT_EQ(mat[9],  0.0);
	EXPECT_EQ(mat[10], 1.0);
	EXPECT_EQ(mat[11], 0.0);
	EXPECT_EQ(mat[12], 0.0);
	EXPECT_EQ(mat[13], 0.0);
	EXPECT_EQ(mat[14], 0.0);
	EXPECT_EQ(mat[15], 1.0);

	float vec[3]; vec[0] = 0; vec[1] = 0; vec[2] = 1;
	test1.loadIdentity();
	test1.rotate(M_PI * 0.5, 0.0, 1.0, 0.0);
	test1.rotateVector(vec);
	EXPECT_NEAR(vec[0], 1.0, 0.001);
	EXPECT_NEAR(vec[1], 0.0, 0.001);
	EXPECT_NEAR(vec[2], 0.0, 0.001);

	test2.loadIdentity();
	test1.rotate(M_PI * 0.5, 0.0, 0.0, 1.0);
	EXPECT_NEAR(test1.getAngleBetween(test2), M_PI * 0.5, 0.001);

	test1.loadIdentity();
	test1.rotate(M_PI * 0.75, 0.0, 1.0, 0.0);
	test2.loadIdentity();
	test2.rotate(M_PI * 0.25, 0.0, 1.0, 0.0);

	vec[0] = 0; vec[1] = 0; vec[2] = 1;
	test1.quatSlerp(test2, 0.5).rotateVector(vec);
	EXPECT_NEAR(vec[0], 1.0, 0.001);
	EXPECT_NEAR(vec[1], 0.0, 0.001);
	EXPECT_NEAR(vec[2], 0.0, 0.001);
}

#endif
