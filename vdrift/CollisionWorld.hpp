#pragma once

#include "CollisionContact.hpp"
#include "MathVector.hpp"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <OgreVector3.h>

//TODO DynamicsWorld functionally identical to btDiscreteDynamicsWorld, but kept
// as a separate class in case of future needs (buoyancy, etc.)
class DynamicsWorld : public btDiscreteDynamicsWorld {
public:
	DynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* broadphase, btConstraintSolver* constraintSolver,
				  btCollisionConfiguration* collisionConfig)
		: btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfig) { }
	~DynamicsWorld() { }

	// vHits only used for damage calculations; not needed now
};

// Manages aspects of Bullet simulations (collisions, bodies, etc.)
// Based on Stuntrally's COLLISION_WORLD in vdrift/collision_world.h
class CollisionWorld {
public:
	CollisionWorld(class Sim* s);
	~CollisionWorld();

	void clear();

	// Model and Track classes unused, so corresponding methods not implemented
	btRigidBody* addRigidBody(const btRigidBody::btRigidBodyConstructionInfo& info,
							  bool isCar = false, bool collideWithCars = false);

	// Casts a ray into the collision world, and returns for first hit (caster is excluded)
	bool castRay(const MathVector<float, 3>& position,
				 const MathVector<float, 3>& direction, const float length,
				 const btCollisionObject* caster, CollisionContact& contact,
				 class CarDynamics* carDyn, int nWheel, bool ignoreCars) const;

	// World physics
	void update(float dt);
	class CarDynamics* oldDyn;

	class Sim* sim;

//	void setMaxSubSteps(int ms) { maxSubSteps = ms; }
//	void setFixedTimeStep(double ft) { fixedTimeStep = ft; }

	DynamicsWorld* getDynamicsWorld() { return world; }

	// Add shape to CollisionWorld (to be deleted by CollisionWorld)
	void addShape(btCollisionShape* sh) { shapes.push_back(sh); }
	void removeShape(btCollisionShape* sh) { shapes.remove(sh); }

private:
//---- Bullet simulation
	btDefaultCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	bt32BitAxisSweep3* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	DynamicsWorld* world;

//---- Objects for deletion
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btActionInterface*> actions;
	btAlignedObjectArray<btTypedConstraint*> constraints;
	btAlignedObjectArray<btTriangleIndexVertexArray*> meshes;

//---- stepSimulation
	int maxSubSteps;
	float fixedTimeStep;
};
