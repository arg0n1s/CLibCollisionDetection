#include "..\include\CollisionDetection.h"
#include "..\include\OctTree.h"
#include <AgentCluster.h>
#include <Agent.h>
#include <Shape.h>

namespace collision {

	using simobj::Agent;
	using simobj::AgentCluster;
	using simobj::SimObjPtr;
	using octtree::OctTree;
	using octtree::Bounds;
	using simobj::shapes::BoundingBox;
	using Eigen::Vector3d;
	using simobj::ReferenceFrame;
	
	CollisionDetection::CollisionDetection() {
		trees = TreeMap();
	}

	void CollisionDetection::makeTreeFromCluster(SimObjPtr cluster) {
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		TreePtr tree = OctTree<unsigned int>::create(Bounds(-10, -10, -10), Bounds(10, 10, 10), Bounds(0.625, 0.625, 0.625));
		trees.insert(std::make_pair(clsPtr->getId(), tree));

		for (auto agent : clsPtr->getAllAgents()) {
			shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent.second);
			const BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
			//!!! For now: This only works with spheres! -> ellipsoids and cylinders need special treatment!!!
			const Vector3d& position = agtPtr->getPosition(ReferenceFrame::Global);
			Bounds lb(position.x()-(bbx.width/2), position.y() - (bbx.height / 2), position.z() - (bbx.length / 2));
			Bounds ub(position.x() + (bbx.width / 2), position.y() + (bbx.height / 2), position.z() + (bbx.length / 2));
			//std::cout << "inserting agent: \n" << agtPtr->toString() << std::endl;
			tree->insertNode(agtPtr->getId(), lb, ub);
		}
	}

	TreePtr CollisionDetection::getTree(const unsigned int& id) {
		return trees.at(id);
	}
}