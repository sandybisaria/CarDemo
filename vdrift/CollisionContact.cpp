#include "CollisionContact.hpp"

#ifdef COMPILE_UNIT_TESTS
#include <gtest/gtest.h>

TEST(CollisionContact, CollisionContactFunctions) {
	CollisionContact* cc = new CollisionContact();
	cc->set(MathVector<float, 3>(1, 1, 1), MathVector<float, 3>(0, 1, 0), 5, NULL, NULL, NULL);

	EXPECT_FALSE(cc->castRay(MathVector<float, 3>(1, 1, 1), MathVector<float, 3>(0, 1, 0), 5));
	MathVector<float, 3> pos = cc->getPosition(), expected = MathVector<float, 3>(1, 6, 1);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(pos[i], expected[i], 0.0001);
	EXPECT_NEAR(cc->getDepth(), 5, 0.0001);

	EXPECT_TRUE(cc->castRay(MathVector<float, 3>(1, 1, 1), MathVector<float, 3>(0, -1, 0), 5));
	pos = cc->getPosition(), expected = MathVector<float, 3>(1, 6, 1);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(pos[i], expected[i], 0.0001);
	EXPECT_NEAR(cc->getDepth(), -5, 0.0001);

	EXPECT_FALSE(cc->castRay(MathVector<float, 3>(0, 0, 0), MathVector<float, 3>(.5, .5, .5), 5));
	pos = cc->getPosition(), expected = MathVector<float, 3>(2.5, 2.5, 2.5);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(pos[i], expected[i], 0.0001);
	EXPECT_NEAR(cc->getDepth(), 5, 0.0001);

	EXPECT_TRUE(cc->castRay(MathVector<float, 3>(0, 0, 0), MathVector<float, 3>(-.5, -.5, -.5), 10));
	pos = cc->getPosition(), expected = MathVector<float, 3>(2.5, 2.5, 2.5);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(pos[i], expected[i], 0.0001);
	EXPECT_NEAR(cc->getDepth(), -5, 0.0001);

	EXPECT_TRUE(cc->castRay(MathVector<float, 3>(1.25, 1.25, 1.25), MathVector<float, 3>(-.5, -.5, -.5), 10));
	pos = cc->getPosition(), expected = MathVector<float, 3>(2.5, 2.5, 2.5);
	for (int i = 0; i < 3; i++) EXPECT_NEAR(pos[i], expected[i], 0.0001);
	EXPECT_NEAR(cc->getDepth(), -2.5, 0.0001);
}

#endif
