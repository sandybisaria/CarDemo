#include "CollisionWorld.hpp"

#include "../Sim.hpp"

#include "../terrain/ShapeData.hpp"

#include "../util/ToBullet.hpp"
#include "CarDynamics.hpp"

// DynamicsWorld::solveConstraints not implemented (only deals with fluids)

CollisionWorld::CollisionWorld(Sim* s)
	: maxSubSteps(24), fixedTimeStep(1.f / 160.f), oldDyn(0), sim(s) {
	config = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(config);

	btScalar ws = 10000; //TODO Get from scene config?
	broadphase = new bt32BitAxisSweep3(btVector3(-ws, -ws, -ws), btVector3(ws, ws, ws));
	solver = new btSequentialImpulseConstraintSolver();

	world = new DynamicsWorld(dispatcher, broadphase, solver, config);

	world->setGravity(btVector3(0.f, 0.f, -9.81f));
	world->getSolverInfo().m_restitution = 0.0f;
	world->getDispatchInfo().m_enableSPU = true;
	world->setForceUpdateAllAabbs(false);
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
		ShapeData* sd = (ShapeData*)(obj->getUserPointer());
		delete sd;
		delete obj;
	}

	for (i = 0; i < shapes.size(); i++) {
		btCollisionShape* shape = shapes[i];

		if (shape->isCompound()) {
			btCompoundShape* cs = (btCompoundShape*)shape;
			for (c = 0; c < cs->getNumChildShapes(); ++c) {
				delete cs->getChildShape(c);
			}
		}
		delete shape;
	}
	shapes.resize(0);

	for (i = 0; i < meshes.size(); ++i) delete meshes[i];
	meshes.resize(0);

	for (i = 0; i < actions.size(); ++i) world->removeAction(actions[i]);
	actions.resize(0);
}

btRigidBody* CollisionWorld::addRigidBody(const btRigidBody::btRigidBodyConstructionInfo& info, bool isCar,
										  bool collideWithCars) {
	btRigidBody* body = new btRigidBody(info);
	btCollisionShape* shape = body->getCollisionShape();

	#define COL_CAR (1 << 2)
	if (isCar) {
		// Apologize for the magic expression
		world->addRigidBody(body, COL_CAR,
							255 - (!collideWithCars ? COL_CAR : 0));
	} else { world->addRigidBody(body); }

	shapes.push_back(shape);
	return body;
}

struct MyRayResultCallback : public btCollisionWorld::RayResultCallback {
	// Have excluded cam variables
	MyRayResultCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld,
						const btCollisionObject* exclude, bool ignoreCars)
			: mRayFromWorld(rayFromWorld), mRayToWorld(rayToWorld),
			  mExclude(exclude), mIgnoreCars(ignoreCars), mShapeId(0) { }

	btVector3 mRayFromWorld, mRayToWorld;
	btVector3 mHitNormalWorld, mHitPointWorld;

	int mShapeId;
	const btCollisionObject* mExclude;
	bool mIgnoreCars;

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,
									 bool normalInWorldSpace) {
		const btCollisionObject* obj = rayResult.m_collisionObject;
		if (obj == mExclude) { return 1.0; }

		ShapeData* sd = (ShapeData*)(obj->getUserPointer());
		if (sd) {
			if (mIgnoreCars && sd->type == ShapeType::Car) { return 1.0; }

			// Car ignores fluids (in this callback)
			if (sd->type == ShapeType::Fluid) { return 1.0; }
			// Also ignore wheels
			if (sd->type == ShapeType::Wheel) { return 1.0; }
		}

		// Caller will assign value of m_closestHitFraction
		btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

		m_closestHitFraction = rayResult.m_hitFraction;
		m_collisionObject = obj;

		if (!rayResult.m_localShapeInfo) { mShapeId = 0; }
		else { mShapeId = rayResult.m_localShapeInfo->m_shapePart; } // Only for btTriangleMeshShape

		if (normalInWorldSpace) { mHitNormalWorld = rayResult.m_hitNormalLocal; }
		else { mHitNormalWorld = m_collisionObject->getWorldTransform().getBasis() *
								 rayResult.m_hitNormalLocal; }

		mHitPointWorld.setInterpolate3(mRayFromWorld, mRayToWorld, rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

bool CollisionWorld::castRay(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction, const float length,
		 const btCollisionObject* caster, CollisionContact& contact, CarDynamics* carDyn, int nWheel, bool ignoreCars) const {
	btVector3 from = toBulletVector(origin);
	btVector3 to = toBulletVector(origin + direction * length);

	MyRayResultCallback res(from, to, caster, ignoreCars);

	MathVector<float, 3> pos, norm; float dist;
	const TerrainSurface* surf = TerrainSurface::none();
	const btCollisionObject* col = NULL; const Bezier* bzr = NULL;

	world->rayTest(from, to, res);
	bool geometryHit = res.hasHit();
	if (geometryHit) {
		pos = toMathVector<float>(res.mHitPointWorld);
		norm = toMathVector<float>(res.mHitNormalWorld);
		dist = res.m_closestHitFraction * length;
		col = res.m_collisionObject;

		// This whole if-else statement seems to be used for determining the surface the wheel is on
		if (col->isStaticObject()) {
			int ptrU = (long) (col->getCollisionShape()->getUserPointer());
			int su = ptrU & 0xFF00, mtr = ptrU & 0xFF;

			//TODO Will need TerrainData or something to determine what exactly is below the wheel
			if (ptrU) {
				switch (su) {
				case SU_Road:
					surf = sim->getTerrainSurface("Asphalt");
					break;

				case SU_Pipe:
					break;

				case SU_Terrain:
					surf = sim->getTerrainSurface("Asphalt");
					break;

				case SU_Fluid:
					break;

				default:
					surf = sim->getTerrainSurface("Asphalt");
					break;
				}
			} else {
				surf = sim->getTerrainSurface("Asphalt");
			}
		}

		contact.set(pos, norm, dist, surf, bzr, col);
		return true;
	}

	// Should only happen on vehicle roll-over
	contact.set(origin + direction * length, -direction, length, surf, bzr, col);
	return false;
}

void CollisionWorld::update(float dt) {
	if (dt <= 0) { return; }

	// dt must be less than maxSubSteps*fixedTimeStep
	world->stepSimulation(dt, maxSubSteps, fixedTimeStep);

	// The rest of Stuntrally's CollisionWorld::update is not useful for our
	// physics simulation needs.
}
