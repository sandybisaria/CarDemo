#include "CollisionWorld.hpp"

void DynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo) {
	btDiscreteDynamicsWorld::solveConstraints(solverInfo);

	int numManifolds = getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {

	}
}
