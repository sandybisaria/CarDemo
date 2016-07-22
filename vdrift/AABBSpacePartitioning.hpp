#pragma once

#include "AABB.hpp"
#include "MathVector.hpp"

#include <list>

template <typename DATATYPE>
class AABBSpacePartitioningNode {
public:
	void add(DATATYPE& object, const AABB<float>& newAABB) {
		objects.push_back(std::pair<DATATYPE, AABB<float> >(object, newAABB));
		if (objects.size() == 1) boundingBox = newAABB;
		else boundingBox.combineWith(newAABB);
	}

	void optimize() {
		collapseTo(*this);
		//TODO removeDuplicateObjects()
		distributeObjectsToChildren(0);
	}

	// Slow delete
	void remove(DATATYPE& object) {
		typename std::list<typename objectListType::iterator> toDel;

		//if we've got objects, test them
		for (typename objectListType::iterator i = objects.begin(); i != objects.end(); ++i)
			if (i->first == object)
				toDel.push_back(i);

		// Do any deletions
		for (typename std::list <typename objectListType::iterator>::iterator i = toDel.begin(); i != toDel.end(); ++i)
			objects.erase(*i);

		// If we have children, pass it on
		for (typename childrenListType::iterator i = children.begin(); i != children.end(); ++i)
			i->remove(object);
	}

	// Faster delete that uses the supplied AABB to find the object
	void remove(DATATYPE& object, const AABB<float>& objAABB) {
		typename std::list <typename objectListType::iterator> toDel;

		//if we've got objects, test them
		for (typename objectListType::iterator i = objects.begin(); i != objects.end(); ++i)
			if (i->first == object)
				toDel.push_back(i);

		// Do any deletions
		for (typename std::list <typename objectListType::iterator>::iterator i = toDel.begin(); i != toDel.end(); ++i)
			objects.erase(*i);

		// If we have children, pass it on
		for (typename childrenListType::iterator i = children.begin(); i != children.end(); ++i)
			if (i->getBoundingBox().intersects(objAABB))
				i->remove(object, objAABB);
	}

	// Run a query for objects that collide with the given shape
	template <typename T, typename U>
	void query(const T& shape, U& outputList) const {
		// If we've got objects, test them
		for (typename objectListType::const_iterator i = objects.begin(); i != objects.end(); ++i)
			if (i->second.intersects(shape))
				outputList.push_back(i->first);

		// If we have children, test them
		for (typename childrenListType::const_iterator i = children.begin(); i != children.end(); ++i)
			if (i->getBoundingBox().intersect(shape))
				// Our child intersects with the segment; dispatch a query
				i->query(shape, outputList);
	}

	bool isEmpty() const { return objects.empty() && children.empty(); }
	void clear() { objects.clear(); children.clear(); }

	// Traverse entire tree, collecting pointers to all DATATYPE objects
	void getContainedObjects(std::list<DATATYPE*>& outputList) {
		// If we've got objects, add them
		for (typename objectListType::iterator i = objects.begin(); i != objects.end(); ++i)
			outputList.push_back(&i->first);

		// If we have children, add them
		for (typename childrenListType::iterator i = children.begin(); i != children.end(); ++i)
			i->getContainedObjects(outputList);
	}

private:
	// Recursively send all objects and childrens' objects to target node (clearing out everything else)
	void collapseTo(AABBSpacePartitioningNode& collapseTarget) {
		if (this != collapseTarget) {
			for (typename objectListType::iterator i = objects.begin(); i != objects.end(); i++)
				collapseTarget.add(i->first, i->second);
			objects.clear();
		}

		for (typename childrenListType::iterator i = children.begin(); i != children.end(); i++)
			i->collapseTo(collapseTarget);

		children.clear();
	}

	//TODO Implement the removeDuplicateObject methods?

	// Intelligently add new child nodes and parse objects to them, recursively
	void distributeObjectsToChildren(const int level) {
		const unsigned int idealObjsPerNode(1), idealChildrenPerNode(2);
		const bool verbose(false);

		// Enforce the rules; start with no children
		for (typename childrenListType::iterator i = children.begin(); i != children.end(); i++)
			i->collapseTo(*this);
		children.clear();

		if (objects.size() <= idealObjsPerNode) return; // No need to distribute

		children.resize(idealChildrenPerNode);

		// Determine average center of all objects
		MathVector<float, 3> avgCenter;
		int numObj = objects.size();
		float incamount = 1.0 / numObj;
		for (typename objectListType::iterator i = objects.begin(); i != objects.end(); i++)
			avgCenter = avgCenter + i->second.getCenter() * incamount;

		// Find axis of max change (for splitting point)
		MathVector<float, 3> axisMask; axisMask.set(1,0,0);
		MathVector<float, 3> boxSize = boundingBox.getSize();
		if (boxSize[0] > boxSize[1] && boxSize[0] > boxSize[2]) axisMask.set(1,0,0);
		else if (boxSize[1] > boxSize[0] && boxSize[1] > boxSize[2]) axisMask.set(0,1,0);
		else if (boxSize[2] > boxSize[1] && boxSize[2] > boxSize[0]) axisMask.set(0,0,1);

		// Distribute objects to each child
		float avgCenterCoord = avgCenter.dot(axisMask);
		int distributor(0);
		for (typename objectListType::iterator i = objects.begin(); i != objects.end(); ++i) {
			float objCenterCoord = i->second.getCenter().dot(axisMask);

			if (objCenterCoord > avgCenterCoord)
				children.front().add(i->first, i->second);
			else if (objCenterCoord < avgCenterCoord)
				children.back().add(i->first, i->second);
			else {
				// Distribute children that sit right on our average center in an even way
				if (distributor % 2 == 0)
					children.front().add(i->first, i->second);
				else
					children.back().add(i->first, i->second);
				distributor++;
			}
		}

		// We've given away all of our objects; clear them out
		objects.clear();

		// Count objects that belong to our children
		int child1Objs = children.front().objects.size();
		int child2Objs = children.back().objects.size();

		if (verbose)
			std::cout << "Objects: " << objects.size() << ", Child nodes: " << children.size() << " L obj: " << child1Objs << " R obj: " << child2Objs << std::endl;

		// If one child doesn't have any objects, then delete both children and take back their objects
		if (child1Objs == 0 || child2Objs == 0) {
			for (typename childrenListType::iterator i = children.begin(); i != children.end(); ++i) {
				for (typename objectListType::iterator n = i->objects.begin(); n != i->objects.end(); ++n)	{
					add(n->first, n->second);
				}
			}
			children.clear();
		}

		for (typename childrenListType::iterator i = children.begin(); i != children.end(); ++i)
			i->distributeObjectsToChildren(level + 1);
	}

	typedef std::vector<std::pair<DATATYPE, AABB<float> > > objectListType;
	objectListType objects;
	typedef std::vector<AABBSpacePartitioningNode> childrenListType;
	childrenListType children;
	AABB<float> boundingBox;
};
