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

	bool CollisionDetection::checkForCollision(SimObjPtr cluster, const IDSet& ignoreIDs, SimObjPtr candidate, SimObjPtr& nearest, double& nearestDistance) {
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		shared_ptr<Agent> candidatePtr = std::static_pointer_cast<Agent>(candidate);

		TreePtr tree = trees.at(clsPtr->getId());

		const Vector3d& position = candidatePtr->getPosition(ReferenceFrame::Global);
		NodePtr<unsigned int> octant = tree->getNearest(position.x(), position.y(), position.z(), ignoreIDs);
		IdSet<unsigned int> candidates = octant->getIds();

		nearest = nullptr;
		nearestDistance = octtree::POS_INF;
		bool collision = false;

		if (candidates.size() == 0) {
			std::cout << "Beep!" << std::endl;
			return collision;
		}
		for (auto id : candidates) {
			shared_ptr<Agent> candidatePtr2 = std::static_pointer_cast<Agent>(clsPtr->getAgent(id));
			if (ignoreIDs.find(candidatePtr2->getId()) != ignoreIDs.end()) continue;
			double tempDistance = calcBodyToBodyDistance(candidatePtr, candidatePtr2);
			if (tempDistance < 0) {
				std::cout << "Boop!" << std::endl;
				collision = true;
				nearestDistance = tempDistance;
				nearest = candidatePtr2;
				std::cout << nearest->getId()<<std::endl;
				return collision;
			}
			
			if (tempDistance < nearestDistance) {
				std::cout << "Baap!" << std::endl;
				nearestDistance = tempDistance;
				nearest = candidatePtr2;
			}
		}
		return collision;
	}

	const double CollisionDetection::calcBodyToBodyDistance(SimObjPtr body1, SimObjPtr body2) const {
		shared_ptr<Agent> bodyPtr1 = std::static_pointer_cast<Agent>(body1);
		shared_ptr<Agent> bodyPtr2 = std::static_pointer_cast<Agent>(body2);
		ShapePtr shape1 = bodyPtr1->getShape();
		ShapeType shapeType1 = shape1->getType();
		ShapePtr shape2 = bodyPtr2->getShape();
		ShapeType shapeType2 = shape2->getType();
		if (shapeType1 == ShapeType::Sphere && shapeType2 == ShapeType::Sphere) {
			return calcSphereToSphereDistance(bodyPtr1, bodyPtr2);
		}
		else if (shapeType1 == ShapeType::Sphere && shapeType2 == ShapeType::Cylinder) {
			return calcSphereToCylinderDistance(bodyPtr1, bodyPtr2);
		}
		else if (shapeType1 == ShapeType::Cylinder && shapeType2 == ShapeType::Sphere) {
			return calcSphereToCylinderDistance(bodyPtr2, bodyPtr1);
		}
		// for the moment ellipsoids are not supported
		else if (shapeType1 == ShapeType::Ellipsoid || shapeType2 == ShapeType::Ellipsoid) {
			return NAN;
		}
		else {
			return calcCylinderToCylinderDistance(bodyPtr1, bodyPtr2);
		}
	}

	const double CollisionDetection::calcSphereToSphereDistance(SimObjPtr sphere1, SimObjPtr sphere2) const {
		std::cout << "Sphere2Sphere action!" << std::endl;
		double distance = octtree::POS_INF;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(sphere1);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(sphere2);
		shared_ptr<Sphere> shape1 = std::static_pointer_cast<Sphere>(agnt1->getShape());
		shared_ptr<Sphere> shape2 = std::static_pointer_cast<Sphere>(agnt2->getShape());
		distance = (agnt1->getPosition(ReferenceFrame::Global) - agnt2->getPosition(ReferenceFrame::Global)).norm();
		distance = distance - (shape1->getRadius() + shape2->getRadius());
		return distance;
	}

	const double CollisionDetection::calcSphereToCylinderDistance(SimObjPtr sphere, SimObjPtr cylinder) const {
		std::cout << "Sphere2Cylinder action!" << std::endl;
		double distance = octtree::POS_INF;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(sphere);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(cylinder);
		shared_ptr<Sphere> shape1 = std::static_pointer_cast<Sphere>(agnt1->getShape());
		shared_ptr<Cylinder> shape2 = std::static_pointer_cast<Cylinder>(agnt2->getShape());
		Vector3d cylinderToSphere = agnt1->getPosition(ReferenceFrame::Global) - agnt2->getPosition(ReferenceFrame::Global);
		Vector3d cylinderRotationalAxis = agnt2->getOrientation(ReferenceFrame::Global)*Vector3d(0, 0, 1);
		std::cout << "sphere origin: " << agnt1->getPosition(ReferenceFrame::Global) << std::endl;
		std::cout << "cylinder origin: " << agnt2->getPosition(ReferenceFrame::Global) << std::endl;
		double alpha = cylinderToSphere.dot(cylinderRotationalAxis);
		double a = std::sin(alpha)*cylinderToSphere.norm();
		std::cout << "angle: " << alpha << std::endl;
		std::cout << "a: " << a << std::endl;
		Vector3d A = cylinderRotationalAxis.normalized()*a;
		Vector3d S = agnt2->getPosition(ReferenceFrame::Global) + A;
		Vector3d cylinderRotAxisToSphere = agnt1->getPosition(ReferenceFrame::Global) - S;
		double verticalDistance = cylinderRotAxisToSphere.norm();
		std::cout << "Vertical Distance: "<< verticalDistance << std::endl;
		distance = verticalDistance - (shape1->getRadius() + shape2->getRadius());
		std::cout << "distance: " << distance << std::endl;
		if (distance > 0) {
			return distance;
		}
		if (a <= shape2->getLength() / 2) return distance;
		std::cout << "a: " << a << std::endl;
		distance = a - (shape2->getLength() / 2 + shape1->getRadius());
		return distance;
	}

	const double CollisionDetection::calcCylinderToCylinderDistance(SimObjPtr cylinder1, SimObjPtr cylinder2) const {
		std::cout << "Cylinder2Cylinder action!" << std::endl;
		double distance = octtree::POS_INF;
		return distance;
	}
}