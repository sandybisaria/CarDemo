#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "MathVector.hpp"
#include "CarDynamics.hpp"
class CarDynamics;
#include "CollisionContact.hpp"

#include <OgreVector3.h>

class DynamicsWorld
	: public btDiscreteDynamicsWorld {
public:
	DynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* broadphase, btConstraintSolver* constraintSolver,
				  btCollisionConfiguration* collisionConfig)
		: btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfig) { }

	~DynamicsWorld() { }

	void solveConstraints(btContactSolverInfo& solverInfo);

	struct Hit
	{
		btVector3 pos, norm, vel;  btScalar force;
		class ShapeData* sdCar;  int dyn;
	};
	btAlignedObjectArray<Hit> vHits;
};

// Manages aspects of Bullet simulations (collisions, bodies, etc.)
// Based on Stuntrally's COLLISION_WORLD in vdrift/collision_world.h
class CollisionWorld {
public:
	CollisionWorld();
	~CollisionWorld();

	void clear();

//	btCollisionObject* addCollisionObject(const Model& model); //FIXME Implement Model class
	btRigidBody* addRigidBody(const btRigidBody::btRigidBodyConstructionInfo& info, bool isCar = false, bool collideWithCars = false);
//	void setTrack(TRACK* t); //FIXME Implement Track or RoadNetwork class

	// Casts a ray into the collision world, and returns for first hit (caster is excluded)
	bool castRay(const MathVector<float, 3>& position, const MathVector<float, 3>& direction, const float length,
				 const btCollisionObject* caster, CollisionContact& contact,
				 CarDynamics* carDyn, int nWheel, bool ignoreCars/*, bool camTilt, bool camDist = false*/) const; //TODO Okay to ignore cam bools?

	// World physics
	void update(double dt);
	CarDynamics* oldDyn;

	class Sim* sim;

	void setMaxSubSteps(int ms) { maxSubSteps = ms; }
	void setFixedTimeStep(double ft) { fixedTimeStep = ft; }

	DynamicsWorld* getDynamicsWorld() { return world; }

	void addShape(btCollisionShape* sh) { shapes.push_back(sh); } // Add shape to CollisionWorld (to be deleted by CollisionWorld)
	void removeShape(btCollisionShape* sh) { shapes.remove(sh); }

private:
	// Bullet simulation
	btDefaultCollisionConfiguration* config;
	btCollisionDispatcher* dispatcher;
	bt32BitAxisSweep3* broadphase;
	btSequentialImpulseConstraintSolver* solver;
	DynamicsWorld* world; //TODO Replace with custom DynamicsWorld class

	// Objects for eventual deletion
	btAlignedObjectArray<btCollisionShape*> shapes;
	btAlignedObjectArray<btActionInterface*> actions;
	btAlignedObjectArray<btTypedConstraint*> constraints;
	btAlignedObjectArray<btTriangleIndexVertexArray*> meshes;

	// Remember that timeStep < maxSubSteps * fixedTimeStep (for stepSimulation)
	int maxSubSteps;
	double fixedTimeStep;
};
