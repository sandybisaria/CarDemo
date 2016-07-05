#include "CollisionWorld.hpp"

#include "TerrainSurface.hpp"
#include "Bezier.hpp"
#include "../terrain/ShapeData.hpp"

void DynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo) {
	btDiscreteDynamicsWorld::solveConstraints(solverInfo);

	int numManifolds = getDispatcher()->getNumManifolds();
	for (int i=0; i < numManifolds; ++i) {
		btPersistentManifold* contactManifold =  getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* bA = contactManifold->getBody0();
		const btCollisionObject* bB = contactManifold->getBody1();

		void *pA = bA->getUserPointer(), *pB = bB->getUserPointer();
		{
			ShapeData* sdA = (ShapeData*)pA, *sdB = (ShapeData*)pB, *sdCar=0, *sdFluid=0, *sdWheel=0;
			if (sdA) {
				if (sdA->type == ShapeType::Car) 		sdCar = sdA;
				else if (sdA->type == ShapeType::Fluid) sdFluid = sdA;
				else if (sdA->type == ShapeType::Wheel) sdWheel = sdA;
			}
			if (sdB) {
				if (sdB->type == ShapeType::Car)		sdCar = sdB;
				else if (sdB->type == ShapeType::Fluid) sdFluid = sdB;
				else if (sdB->type == ShapeType::Wheel) sdWheel = sdB;
			}

//			if (sdFluid) { }
//				if (sdWheel) {
//					std::list<FluidBox*>& fl = sdWheel->pCarDyn->inFluidsWh[sdWheel->whNum];
//					if (fl.empty())
//						fl.push_back(sdFluid->pFluid);
//				} else if (sdCar) {
//					if (sdCar->pCarDyn->inFluids.empty())
//						sdCar->pCarDyn->inFluids.push_back(sdFluid->pFluid);
//				}
		}
	}
}

void IntTickCallback(btDynamicsWorld* world, btScalar timeStep) {
	CollisionWorld* cw = (CollisionWorld*)world->getWorldUserInfo();

	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0; i < numManifolds; ++i) {
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* bA = contactManifold->getBody0();
		const btCollisionObject* bB = contactManifold->getBody1();

		if ((bA->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE) ||
			(bB->getCollisionFlags() & btCollisionObject::CF_NO_CONTACT_RESPONSE))
			continue;

		void* pA = bA->getUserPointer(), *pB = bB->getUserPointer();
		{
			ShapeData* sdA = (ShapeData*)pA, *sdB = (ShapeData*)pB, *sdCar=0, *sdFluid=0, *sdWheel=0;
			if (sdA) {
				if (sdA->type == ShapeType::Car) 		sdCar = sdA;
				else if (sdA->type == ShapeType::Fluid) sdFluid = sdA;
				else if (sdA->type == ShapeType::Wheel) sdWheel = sdA;
			}
			if (sdB) {
				if (sdB->type == ShapeType::Car)		sdCar = sdB;
				else if (sdB->type == ShapeType::Fluid) sdFluid = sdB;
				else if (sdB->type == ShapeType::Wheel) sdWheel = sdB;
			}

//			if (sdFluid && sdFluid->pFluid)  // Solid fluid hit TODO Fluids
//				if (sdFluid->pFluid->solid)  sdFluid = 0;

			if (sdCar && !sdFluid && !sdWheel) {
				bool dyn = (sdCar == sdA && !bB->isStaticObject()) || (sdCar == sdB && !bA->isStaticObject());
				int num = contactManifold->getNumContacts();
				for (int j=0; j < num; ++j) {
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					btScalar f = pt.getAppliedImpulse() * timeStep;
					if (f > 0.f) {
						DynamicsWorld::Hit hit;
						hit.dyn = dyn ? 1 : 0;  //TODO Dev suggested custom sound for object type
						hit.pos = pt.getPositionWorldOnA();
						hit.norm = pt.m_normalWorldOnB;
						hit.force = f;  hit.sdCar = sdCar;
						hit.vel = sdCar ? sdCar->dyn->prevVel : (btVector3(1, 1, 1) * 0.1f);
						cw->getDynamicsWorld()->vHits.push_back(hit);
					}
				}
			}
		}
	}
}

CollisionWorld::CollisionWorld()
	: maxSubSteps(24), fixedTimeStep(1. / 160.), oldDyn(0), sim(0) /*Defaults according to Stuntrally settings*/ {
	config = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(config);

	btScalar ws = 10000;
	broadphase = new bt32BitAxisSweep3(btVector3(-ws, -ws, -ws), btVector3(ws, ws, ws));
	solver = new btSequentialImpulseConstraintSolver();

	world = new DynamicsWorld(dispatcher, broadphase, solver, config);

	world->setGravity(btVector3(0., 0., -9.81));
	world->getSolverInfo().m_restitution = 0.0f;
	world->getDispatchInfo().m_enableSPU = true;

	world->setInternalTickCallback(IntTickCallback, this, false);
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
			: mRayFromWorld(rayFromWorld), mRayToWorld(rayToWorld), mExclude(exclude), mIgnoreCars(ignoreCars),
			  mShapeId(0) {}

	btVector3 mRayFromWorld, mRayToWorld;
	btVector3 mHitNormalWorld, mHitPointWorld;

	int mShapeId;

	const btCollisionObject* mExclude;
	bool mIgnoreCars;

	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) {
		const btCollisionObject* obj = rayResult.m_collisionObject;
		if (obj == mExclude) return 1.0;

		ShapeData* sd = (ShapeData*)(obj->getUserPointer());
		if (sd) {
			if (mIgnoreCars && sd->type == ShapeType::Car) return 1.0;

			// Car ignores fluids (Not dealing with cars)
			if (sd->type == ShapeType::Fluid) return 1.0;

			// Ignore wheel triggers here
			if (sd->type == ShapeType::Wheel) return 1.0;
		}

		// Caller will assign value of m_closestHitFraction
		btAssert(rayResult.m_hitFraction <= m_closestHitFraction);

		m_closestHitFraction = rayResult.m_hitFraction;
		m_collisionObject = obj;

		if (!rayResult.m_localShapeInfo)
			mShapeId = 0;  // Crash hf-
		else  // Only for btTriangleMeshShape
			mShapeId = rayResult.m_localShapeInfo->m_shapePart;

		if (normalInWorldSpace) mHitNormalWorld = rayResult.m_hitNormalLocal;
		else mHitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;

		mHitPointWorld.setInterpolate3(mRayFromWorld, mRayToWorld, rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

bool CollisionWorld::castRay(const MathVector<float, 3>& position, const MathVector<float, 3>& direction, const float length,
		 const btCollisionObject* caster, CollisionContact& contact, CarDynamics* carDyn, int nWheel, bool ignoreCars) const {
	btVector3 from = toBulletVector(position);
	btVector3 to = toBulletVector(position + direction * length);

	MyRayResultCallback res(from, to, caster, ignoreCars);

	MathVector<float, 3> pos, norm;
	float dist;
	const TerrainSurface* surf = TerrainSurface::none();
	const btCollisionObject* col = NULL;
	const Bezier* bzr = NULL;

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
			int su = ptrU & 0xFF00, mtr = ptrU & 0xFF; // Fancy arithmetic

			//TODO Will need TerrainData to determine what exactly is below the wheel
			if (ptrU) {
				switch (su) {
				case SU_Road:
					break;

				case SU_Pipe:
					break;

				case SU_Terrain:
					surf = sim->getTerrainSurface("Default"); //TODO Hard-coded for now
					break;

				case SU_Fluid:
					break;

				default:
					break;
				}
			} else {
				//TODO Go into TerrainData and get the TerrainSurface
				surf = sim->getTerrainSurface("Default"); //TODO Hard-coded for now
			}
		}

		//TODO When Track is implemented, use the Beziers to track collisions, or something...

		contact.set(pos, norm, dist, surf, bzr, col);
		return true;
	}

	// Should only happen on vehicle roll-over
	contact.set(position + direction * length, -direction, length, surf, bzr, col);
	return false;
}

void CollisionWorld::update(double dt) {
	world->stepSimulation(dt, maxSubSteps, fixedTimeStep);

	// Use collision hit results, once a frame
	int n = world->vHits.size();
	if (n > 0) {
		// Pick the one with biggest force
		DynamicsWorld::Hit& hit = world->vHits[0];
		float force = 0.f;//, vel = 0.f;
		for (int i=0; i < n; ++i)
			if (world->vHits[i].force > force) {
				force = world->vHits[i].force;
				hit = world->vHits[i];
			}

		CarDynamics* cd = hit.sdCar->dyn;
		oldDyn = cd;
		btVector3 vcar = hit.vel;
		Ogre::Vector3 vel(vcar[0], vcar[2], -vcar[1]);
		Ogre::Vector3 norm(hit.norm.getX(), hit.norm.getZ(), -hit.norm.getY());
		float vlen = vel.length(), normvel = abs(vel.dotProduct(norm));
	}

	// Ignored cdOld and fHitForce

	world->vHits.clear();
}
