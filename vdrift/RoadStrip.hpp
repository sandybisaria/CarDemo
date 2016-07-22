#pragma once

#include "RoadPatch.hpp"
#include "Optional.hpp"
#include "AABBSpacePartitioning.hpp"

#include <iostream>
#include <list>

class RoadStrip {
public:
	RoadStrip() : closed(false) { }

	bool readFrom(std::istream& file, std::ostream& errorOutput) {
		patches.clear();

		assert(file);

		int num; file >> num; // Number of patches in road strip

		// Add road patches to this strip
		int badCount = 0;
		for (int i = 0; i < num; i++) {
			Bezier* prevBezier = NULL;
			if (!patches.empty()) { prevBezier = &patches.back().getPatch(); }

			patches.push_back(RoadPatch());
			patches.back().getPatch().readFrom(file);

			if (prevBezier) { prevBezier->attach(patches.back().getPatch()); }

			if (patches.back().getPatch().checkForProblems()) {
				badCount++;
				patches.pop_back();
			}
		}

		if (badCount > 0) {
			errorOutput << "Rejected " << badCount << " bezier patch(es) due to errors" << std::endl;
		}

		// Close the road strip
		if (patches.size() > 2) {
			double leftDist = (patches.back().getPatch().getFrontLeft() - patches.front().getPatch().getBackLeft()).magnitude();
			double rightDist = (patches.back().getPatch().getFrontRight() - patches.front().getPatch().getBackRight()).magnitude();

			// Close if first and last patch are close to each other
			if (leftDist < 0.1 && rightDist < 0.1) {
				patches.back().getPatch().attach(patches.front().getPatch());
				closed = true;
			}
		}

		generateSpacePartitioning();

		return true;
	}

	bool collide(const MathVector<float, 3>& origin, const MathVector<float, 3>& direction, float segLen,
				 MathVector<float, 3>& outtri, MathVector<float, 3>& normal, const Bezier*& colPatch) {
		std::list <RoadPatch*> candidates;
		aabbPart.query(AABB<float>::Ray(origin, direction, segLen), candidates);

		bool col = false;
		for (std::list<RoadPatch*>::iterator i = candidates.begin(); i != candidates.end(); i++) {
			MathVector<float, 3> coltri, colnorm;
			if ((*i)->collide(origin, direction, segLen, coltri, colnorm)) {
				// Find closest patch that collides
				if (!col || (coltri - origin).magnitude() < (outtri - origin).magnitude()) {
					outtri = coltri;
					normal = colnorm;
					colPatch = &(*i)->getPatch();
				}
				col = true;
			}
		}

		return col;
	}

	void reverse() {
		patches.reverse();

		for (std::list<RoadPatch>::iterator i = patches.begin(); i != patches.end(); i++) {
			i->getPatch().reverse();
			i->getPatch().resetDistFromStart();
		}

		// Fix pointers to next patch
		for (std::list<RoadPatch>::iterator i = patches.begin(); i != patches.end(); i++) {
			std::list<RoadPatch>::iterator j = i; j++;

			Bezier* nextPatch = NULL;
			if (j != patches.end()) {
				nextPatch = &(j->getPatch());
				i->getPatch().attach(*nextPatch);
			} else {
				i->getPatch().resetNextPatch();
				i->getPatch().attach(patches.front().getPatch());
			}
		}
	}

	const std::list<RoadPatch>& getPatchList() const { return patches; }
		  std::list<RoadPatch>& getPatchList()		 { return patches; }

	Optional<const Bezier*> findBezierAtOffset(const Bezier* bezier, int offset = 0) const {
		std::list<RoadPatch>::const_iterator found = patches.end(); // Holds the found RoadPatch (if any)

		// Search for the road patch containing the bezier
		for (std::list<RoadPatch>::const_iterator i = patches.begin(); i != patches.end(); i++) {
			if (&i->getPatch() == bezier) {
				found = i;
				break;
			}
		}

		if (found == patches.end()) { return Optional<const Bezier*>(); }

		int curOffset = offset;
		while (curOffset != 0) {
			if (curOffset < 0) {
				// Why is this so difficult?  all i'm trying to do is make the iterator loop around (straight from the source...)
				std::list<RoadPatch>::const_reverse_iterator rit(found);
				if (rit == patches.rend())
					rit = patches.rbegin();
				rit++;
				if (rit == patches.rend())
					rit = patches.rbegin();
				found = rit.base();
				if (found == patches.end())
					found = patches.begin();

				++curOffset;
			} else {
				found++;
				if (found == patches.end()) { found = patches.begin(); }

				curOffset--;
			}
		}

		assert(found != patches.end());
		return Optional<const Bezier*>(&found->getPatch());
	}

private:
	void generateSpacePartitioning() {
		aabbPart.clear();

		for (std::list<RoadPatch>::iterator i = patches.begin(); i != patches.end(); i++) {
			RoadPatch* rp = &(*i);
			aabbPart.add(rp, i->getPatch().getAABB());
		}

		aabbPart.optimize();
	}

	bool closed;
	std::list<RoadPatch> patches;
	AABBSpacePartitioningNode <RoadPatch*> aabbPart;
};