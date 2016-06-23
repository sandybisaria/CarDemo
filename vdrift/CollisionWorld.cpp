#include "CollisionWorld.hpp"

#include "TerrainSurface.hpp"

CollisionWorld::CollisionWorld()
	: maxSubSteps(24), fixedTimeStep(1. / 160.) /*Defaults according to Stuntrally settings*/ {
	config = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(config);

	btScalar ws = 10000;
	broadphase = new bt32BitAxisSweep3(btVector3(-ws, -ws, -ws), btVector3(ws, ws, ws));
	solver = new btSequentialImpulseConstraintSolver();

	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);

	world->setGravity(btVector3(0., 0., -9.81));
	world->getSolverInfo().m_restitution = 0.0f;
	world->getDispatchInfo().m_enableSPU = true;

//	world->setInternalTickCallback(IntTickCallback, this, false); //TODO When ready for fluids..
}

CollisionWorld::~CollisionWorld() {
	clear();

	delete world;
	delete solver;
	delete broadphase;
	delete dispatcher;
	delete config;
}

void CollisionWorld::clear() {
	oldDyn = NULL;

	// Remove constraints first
	int i, c;
	for (i = 0; i < constraints.size(); i++) {
		world->removeConstraint(constraints[i]);
		delete constraints[i];
	}
	constraints.resize(0);

	for (i = world->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = world->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) delete body->getMotionState();

		world->removeCollisionObject(obj);
		//TODO Get ShapeData from obj and delete
		delete obj;
	}

	for (i = 0; i < shapes.size(); i++) {
		btCollisionShape* shape = shapes[i];
		if (shape->isCompound()) {
			btCompoundShape* cs = (btCompoundShape*)shape;
			for (c = 0; c < cs->getNumChildShapes(); ++c) delete cs->getChildShape(c);
		}
		delete shape;
	}
	shapes.resize(0);

	for (i = 0; i < meshes.size(); ++i) delete meshes[i];
	meshes.resize(0);

	for (i = 0; i < actions.size(); ++i) world->removeAction(actions[i]);
	actions.resize(0);
}

btRigidBody* CollisionWorld::addRigidBody(const btRigidBody::btRigidBodyConstructionInfo& info, bool isCar, bool collideWithCars) {
	btRigidBody* body = new btRigidBody(info);
	btCollisionShape* shape = body->getCollisionShape();

	#define COL_CAR (1 << 2)
	if (isCar) world->addRigidBody(body, COL_CAR, 255 - (!collideWithCars ? COL_CAR : 0));
	else world->addRigidBody(body);

	shapes.push_back(shape);
	return body;
}

struct MyRayResultCallback
	: public btCollisionWorld::RayResultCallback {
	//TODO Include cam variables?
	MyRayResultCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld, const btCollisionObject* exclude, bool ignoreCars)
			: mRayFromWorld(rayFromWorld), mRayToWorld(rayToWorld), mExclude(exclude), mIgnoreCars(ignoreCars) {}

	btVector3 mRayFromWorld, mRayToWorld;
	btVector3 mHitNormalWorld, mHitPointWorld;

	const btCollisionObject* mExclude;
	bool mIgnoreCars;

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) {
		const btCollisionObject* obj = rayResult.m_collisionObject;
		if (obj == mExclude) return 1.0;

		//FIXME Include ShapeData for car detection!

		// Caller will assign value of m_closestHitFraction
		btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

		m_closestHitFraction = rayResult.m_hitFraction;
		m_collisionObject = obj;

		//TODO Add m_shapeId member?

		if (normalInWorldSpace) mHitNormalWorld = rayResult.m_hitNormalLocal;
		else mHitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;

		mHitPointWorld.setInterpolate3(mRayFromWorld, mRayToWorld, rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

bool CollisionWorld::castRay(const MathVector<float, 3>& position, const MathVector<float, 3>& direction, const float length,
		 const btCollisionObject* caster, CarDynamics* carDyn, int nWheel, bool ignoreCars) const {
	btVector3 from = toBulletVector(position);
	btVector3 to = toBulletVector(position + direction * length);

	MyRayResultCallback res(from, to, caster, ignoreCars);

	MathVector<float, 3> pos, norm;
	float dist;
	const TerrainSurface* surf = TerrainSurface::none();
	const btCollisionObject* col = NULL;
//	const Bezier* bzr = NULL; //FIXME Add Bezier class?

	world->rayTest(from, to, res);
	bool geometryHit = res.hasHit();
	if (geometryHit) {
		pos = toMathVector<float>(res.mHitPointWorld);
		norm = toMathVector<float>(res.mHitNormalWorld);
		dist = res.m_closestHitFraction * length;
		col = res.m_collisionObject;
		//TODO Get TerrainData (TerData)

		if (col->isStaticObject()) {
			int ptrU = (long) (col->getCollisionShape()->getUserPointer());
			int su = ptrU & 0xFF00, mtr = ptrU & 0xFF; // Fancy arithmetic

			if (ptrU) {
				switch (su) {
				//TODO Go through different SU cases, defined in ShapeData.h
				}
			} else {
				//TODO Go into TerrainData and get the TerrainSurface
			}
		}
		//TODO When Track is implemented, use the Beziers to track collisions, or something...

		//TODO Set CollisionContact
		return true;
	}

	return false;
}

void CollisionWorld::update(double dt) {
	world->stepSimulation(dt, maxSubSteps, fixedTimeStep);

	//TODO Do something with fluids... as usual

	//TODO If you have oldDyn, modify some fHitForce variables...
}
