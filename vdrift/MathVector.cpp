#include "MathVector.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

#include <cmath>
#define _USE_MATH_DEFINES

TEST(MathVector, MathVectorBasic) {
	MathVector<int, 3> test1(8);
	MathVector<int, 3> test2(2); test2.set(5);

	MathVector<int, 3> test3; test3 = test1 + test2;
	EXPECT_EQ(test3[0], 13);
}

TEST(MathVector, MathVectorSubtraction) {
	MathVector<float, 2> test1(0); test1.set(3, 4);
	MathVector<float, 2> test2(1);

	MathVector<float, 2> test3(test1 - test2);
	EXPECT_EQ(test3[0], 2);
	EXPECT_EQ(test3[1], 3);
}

TEST(MathVector, MathVectorEquality) {
	MathVector<float, 3> test1; test1.set(1, 2, 3);
	MathVector<float, 3> test2(1);
	MathVector<float, 3> test3(test1 + test2);
	EXPECT_EQ(test3[0], 2);
	EXPECT_EQ(test3[1], 3);
	EXPECT_EQ(test3[2], 4);

	test3 = test1;
	EXPECT_TRUE(test1 == test3);
	EXPECT_TRUE(test1 != test2);

	test3 = -test1;
	MathVector<float, 3> answer;
	answer.set(-1, -2, -3);
	EXPECT_EQ(test3,answer);
}

TEST(MathVector, MathVectorOperations) {
	MathVector<float, 3> test1; test1.set(1,2,3);
	MathVector<float, 3> testcopy(test1);
	EXPECT_EQ(test1, testcopy);

	float v3[3];
	for (int i = 0; i < 3; ++i)	v3[i] = i + 1;
	testcopy.set(v3);
	EXPECT_EQ(test1, testcopy);

	MathVector<float,3> add1; add1.set(1);
	for (int i = 0; i < 3; ++i) testcopy[i] = i;
	testcopy = testcopy + add1;
	EXPECT_EQ(test1, testcopy);

	for (int i = 0; i < 3; ++i) testcopy[i] = i + 2;
	testcopy = testcopy - add1;
	EXPECT_EQ(test1, testcopy);

	testcopy = testcopy * 1.0;
	EXPECT_EQ(test1, testcopy);

	testcopy = testcopy / 1.0;
	EXPECT_EQ(test1, testcopy);
	EXPECT_TRUE(test1 == testcopy);
	EXPECT_TRUE(!(test1 == add1));
	EXPECT_TRUE(test1 != add1);
	EXPECT_TRUE(!(test1 != testcopy));

	for (int i = 0; i < 3; ++i)	testcopy[i] = -(i + 1);
	testcopy = -testcopy;
	MathVector<float, 3> zero(0);
	for (int i = 0; i < 3; ++i) EXPECT_EQ(zero[i], 0);
	EXPECT_EQ(test1, testcopy);

	testcopy.set(0.0);
	EXPECT_EQ(testcopy, zero);

	testcopy = test1;
	EXPECT_EQ(testcopy, test1);
	EXPECT_NEAR(test1.magnitudeSquared(), 14.0, 0.001);
	EXPECT_NEAR(test1.magnitude(), 3.741657, 0.001);
	EXPECT_NEAR(test1.normalized()[0], 0.267261, 0.001);
	EXPECT_NEAR(test1.normalized()[1], 0.534522, 0.001);

	MathVector<float,3> test2;
	for (int i = 0; i < 3; ++i) EXPECT_EQ(test2[i], 0);
	test2.set(2,3,4);
	EXPECT_NEAR(test1.dot(test2), 20.0, 0.001);

	test1.set(1,-1,-2);
	test1.toAbsVal();
	EXPECT_EQ(test1, (MathVector<float, 3>(1,1,2)));
}

TEST(MathVector, MathVectorCross) {
	MathVector<float, 3> test1; test1.set(1,2,3);
	MathVector<float, 3> test2; test2.set(4,5,6);
	MathVector<float, 3> answer; answer.set(-3,6,-3);
	EXPECT_EQ(test1.cross(test2), answer);
}

#endif
