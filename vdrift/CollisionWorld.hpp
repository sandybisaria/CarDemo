#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

class DynamicsWorld : public btDiscreteDynamicsWorld {
public:
	DynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* broadphase, btConstraintSolver* constraintSolver,
				  btCollisionConfiguration* collisionConfig)
		: btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfig) { }
	~DynamicsWorld() { }

	void solveConstraints(btContactSolverInfo& solverInfo);

	struct Hit {
		btVector3 pos, norm, vel;
		btScalar force;
		class ShapeData* sdCar;
		int dyn;
	};
	btAlignedObjectArray<Hit> vHits;
};

// Manages bodies, collision objects/shapes
// Based on COLLISION_WORLD in vdrift/collision_world.h
class CollisionWorld {

};
