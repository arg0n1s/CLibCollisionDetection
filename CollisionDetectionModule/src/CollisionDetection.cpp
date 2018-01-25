#undef max
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
		initialTreeDiameter = 16;
		minimalCellDiameter = 2;
		allowRescaling = false;
	}

	void CollisionDetection::setInitialTreeDiameter(const double& initialTreeDiameter) {
		this->initialTreeDiameter = initialTreeDiameter;
	}

	void CollisionDetection::setMinimalCellDiameter(const double& minimalCellDiameter) {
		this->minimalCellDiameter = minimalCellDiameter;
	}

	void CollisionDetection::setAllowRescaling(const bool rescalingOn) {
		this->allowRescaling = rescalingOn;
	}

	void CollisionDetection::makeTreeFromCluster(SimObjPtr cluster) {
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		TreePtr tree = OctTree<unsigned int>::create(Bounds(initialTreeDiameter, initialTreeDiameter, initialTreeDiameter), Bounds(minimalCellDiameter, minimalCellDiameter, minimalCellDiameter));
		tree->setAllowResize(allowRescaling);
		trees.insert(std::make_pair(clsPtr->getId(), tree));

		for (auto agent : clsPtr->getAllAgents()) {
			shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent.second);
			const BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
			const Vector3d& position = agtPtr->getPosition(ReferenceFrame::Global);
			double halfMaxDim = std::max(bbx.width, std::max(bbx.height, bbx.length)) / 2.0;
			Bounds lb(position.x()- halfMaxDim, position.y() - halfMaxDim, position.z() - halfMaxDim);
			Bounds ub(position.x() + halfMaxDim, position.y() + halfMaxDim, position.z() + halfMaxDim);
			tree->insertNode(agtPtr->getId(), lb, ub);
		}
	}

	TreePtr CollisionDetection::getTree(const unsigned int& id) {
		return trees.at(id);
	}
}