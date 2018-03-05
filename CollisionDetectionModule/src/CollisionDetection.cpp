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
	using octtree::IdSet;
	using octtree::NodePtr;
	using simobj::shapes::ShapePtr;
	using simobj::shapes::ShapeType;
	using simobj::shapes::Sphere;
	using simobj::shapes::Cylinder;
	using simobj::shapes::Ellipsoid;

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
			Bounds lb(position.x() - halfMaxDim, position.y() - halfMaxDim, position.z() - halfMaxDim);
			Bounds ub(position.x() + halfMaxDim, position.y() + halfMaxDim, position.z() + halfMaxDim);
			tree->insertNode(agtPtr->getId(), lb, ub);
		}
	}

	TreePtr CollisionDetection::getTree(const unsigned int& id) {
		return trees.at(id);
	}

	bool CollisionDetection::checkForCollision(SimObjPtr cluster, const unsigned int ignoreID, SimObjPtr candidate, SimObjPtr nearest, double& nearestDistance) {
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		shared_ptr<Agent> candidatePtr = std::static_pointer_cast<Agent>(candidate);
		ShapePtr candidateShape = candidatePtr->getShape();
		ShapeType candidateType = candidateShape->getType();
		TreePtr tree = trees.at(clsPtr->getId());
		IdSet<unsigned int> idSet;
		idSet.insert(ignoreID);
		const Vector3d& position = candidatePtr->getPosition(ReferenceFrame::Global);
		NodePtr<unsigned int> octant = tree->getNearest(position.x(), position.y(), position.z(), idSet);
		IdSet<unsigned int> candidates = octant->getIds();
		nearest = nullptr;
		nearestDistance = octtree::POS_INF;
		bool collision = false;
		if (candidates.size() == 0) {
			return collision;
		}
		for (auto id : candidates) {
			shared_ptr<Agent> candidatePtr2 = std::static_pointer_cast<Agent>(clsPtr->getAgent(id));
			if (candidatePtr2->getId() == ignoreID) continue;
			double tempDistance = 0.0;
			bool tempCollision = false;
			ShapePtr candidateShape2 = candidatePtr2->getShape();
			ShapeType candidateType2 = candidateShape2->getType();
			if (candidateType == ShapeType::Sphere && candidateType2 == ShapeType::Sphere) {
				tempCollision = checkSphereOnSphereCollision(candidatePtr, candidatePtr2, tempDistance);
			}
			else if (candidateType == ShapeType::Sphere && candidateType2 == ShapeType::Cylinder) {
				tempCollision = checkSphereOnCylinderCollision(candidatePtr, candidatePtr2, tempDistance);
			}
			else if (candidateType == ShapeType::Cylinder && candidateType2 == ShapeType::Sphere) {
				tempCollision = checkSphereOnCylinderCollision(candidatePtr2, candidatePtr, tempDistance);
			}
			// for the moment ellipsoids are not supported
			else if (candidateType == ShapeType::Ellipsoid || candidateType2 == ShapeType::Ellipsoid) {
				continue;
			}
			else {
				tempCollision = checkCylinderOnCylinderCollision(candidatePtr, candidatePtr2, tempDistance);
			}
			if (tempCollision && tempDistance < nearestDistance) {
				collision = true;
				nearestDistance = tempDistance;
				nearest = candidatePtr2;
			}
		}
		return collision;
	}

	bool CollisionDetection::checkSphereOnSphereCollision(SimObjPtr sphere1, SimObjPtr sphere2, double& distance) {
		bool collision = false;
		distance = octtree::POS_INF;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(sphere1);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(sphere2);
		shared_ptr<Sphere> shape1 = std::static_pointer_cast<Sphere>(agnt1->getShape());
		shared_ptr<Sphere> shape2 = std::static_pointer_cast<Sphere>(agnt2->getShape());
		distance = (agnt1->getPosition(ReferenceFrame::Global) - agnt2->getPosition(ReferenceFrame::Global)).norm();
		distance = distance - (shape1->getRadius() + shape2->getRadius());
		if (distance <= 0) collision = true;
		return collision;

	}
	bool CollisionDetection::checkSphereOnCylinderCollision(SimObjPtr sphere, SimObjPtr cylinder, double& distance) {
		bool collision = false;
		distance = octtree::POS_INF;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(sphere);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(cylinder);
		shared_ptr<Sphere> shape1 = std::static_pointer_cast<Sphere>(agnt1->getShape());
		shared_ptr<Cylinder> shape2 = std::static_pointer_cast<Cylinder>(agnt2->getShape());
		Vector3d ZK = agnt1->getPosition(ReferenceFrame::Global) - agnt2->getPosition(ReferenceFrame::Global);
		Vector3d Z = agnt2->getOrientation(ReferenceFrame::Global)*Vector3d(0, 0, 1);
		double alpha = ZK.dot(Z);
		double a = std::sin(alpha)*ZK.norm();
		Vector3d A = Z.normalized()*a;
		Vector3d S = agnt2->getPosition(ReferenceFrame::Global) + A;
		Vector3d ZK90 = agnt1->getPosition(ReferenceFrame::Global) - S;
		double zk90 = ZK90.norm();
		distance = zk90 - (shape1->getRadius() + shape2->getRadius());
		if (distance > 0) {
			return collision;
		}
		distance = a - (shape2->getLength() / 2 + shape1->getRadius());
		if (distance <= 0) collision = true;
		return collision;
	}
	bool CollisionDetection::checkCylinderOnCylinderCollision(SimObjPtr cylinder1, SimObjPtr cylinder2, double& distance) {
		bool collision = false;
		distance = octtree::POS_INF;
		return collision;
	}
}