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
	using Eigen::ParametrizedLine;
	using Eigen::Hyperplane;

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

	const bool CollisionDetection::isClusterInTree(const unsigned int& id) const {
		return trees.find(id) != trees.end();
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

	void CollisionDetection::addAgentToTree(SimObjPtr agent) {
		shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent);
		TreePtr tree = trees[(agtPtr->getAgentCluster()->getId())];
		const BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
		const Vector3d& position = agtPtr->getPosition(ReferenceFrame::Global);
		double halfMaxDim = std::max(bbx.width, std::max(bbx.height, bbx.length)) / 2.0;
		Bounds lb(position.x() - halfMaxDim, position.y() - halfMaxDim, position.z() - halfMaxDim);
		Bounds ub(position.x() + halfMaxDim, position.y() + halfMaxDim, position.z() + halfMaxDim);
		tree->insertNode(agtPtr->getId(), lb, ub);
	}

	TreePtr CollisionDetection::getTree(const unsigned int& id) {
		return trees.at(id);
	}

	const bool CollisionDetection::checkForCollision(SimObjPtr cluster, const IDSet& ignoreIDs, SimObjPtr candidate, SimObjPtr& nearest, double& nearestDistance) {
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
			return collision;
		}
		for (auto id : candidates) {
			shared_ptr<Agent> candidatePtr2 = std::static_pointer_cast<Agent>(clsPtr->getAgent(id));
			if (ignoreIDs.find(candidatePtr2->getId()) != ignoreIDs.end()) continue;
			double tempDistance = calcBodyToBodyDistance(candidatePtr, candidatePtr2);
			if (tempDistance < 0) {
				collision = true;
				nearestDistance = tempDistance;
				nearest = candidatePtr2;
				return collision;
			}
			
			if (tempDistance < nearestDistance) {
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
		double distance = octtree::POS_INF;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(sphere);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(cylinder);
		shared_ptr<Sphere> shape1 = std::static_pointer_cast<Sphere>(agnt1->getShape());
		shared_ptr<Cylinder> shape2 = std::static_pointer_cast<Cylinder>(agnt2->getShape());
		Vector3d cylinderOrientation = agnt2->getOrientation(ReferenceFrame::Global)*Vector3d(0, 0, 1);
		Vector3d cylinderOrigin = agnt2->getPosition(ReferenceFrame::Global);
		Vector3d sphereOrigin = agnt1->getPosition(ReferenceFrame::Global);
		ParametrizedLine<double, 3> cylinderLine(cylinderOrigin, cylinderOrientation);
		double distSphereToLine = cylinderLine.distance(sphereOrigin);
		Vector3d pointSphereOnLine = cylinderLine.projection(sphereOrigin);
		Vector3d originToPoint = pointSphereOnLine - cylinderOrigin;
		double distOriginToPoint = originToPoint.norm();
		if (distOriginToPoint <= shape2->getLength() / 2) {
			distance = distSphereToLine - (shape1->getRadius() + shape2->getRadius());
			return distance;
		}
		Vector3d topOfCylinder = cylinderOrigin + (shape2->getLength() / 2)*originToPoint.normalized();
		Vector3d pointToSphere = sphereOrigin - pointSphereOnLine;
		Vector3d topCornerCylinder = topOfCylinder + shape2->getRadius()*pointToSphere.normalized();
		double distSphereToCorner = (topCornerCylinder - sphereOrigin).norm();
		distance = distSphereToCorner - shape1->getRadius();
		return distance;
	}

	const double CollisionDetection::calcCylinderToCylinderDistance(SimObjPtr cylinder1, SimObjPtr cylinder2) const {
		double distance = octtree::POS_INF;
		double eps = 0.00001;
		shared_ptr<Agent> agnt1 = std::static_pointer_cast<Agent>(cylinder1);
		shared_ptr<Agent> agnt2 = std::static_pointer_cast<Agent>(cylinder2);
		shared_ptr<Cylinder> shape1 = std::static_pointer_cast<Cylinder>(agnt1->getShape());
		shared_ptr<Cylinder> shape2 = std::static_pointer_cast<Cylinder>(agnt2->getShape());
		Vector3d cylinderOrientation1 = (agnt1->getOrientation(ReferenceFrame::Global)*Vector3d(0, 0, 1)).normalized();
		Vector3d cylinderOrigin1 = agnt1->getPosition(ReferenceFrame::Global);
		Vector3d cylinderOrientation2 = (agnt2->getOrientation(ReferenceFrame::Global)*Vector3d(0, 0, 1)).normalized();
		Vector3d cylinderOrigin2 = agnt2->getPosition(ReferenceFrame::Global);
		ParametrizedLine<double, 3> cylinderLine1(cylinderOrigin1, cylinderOrientation1);
		ParametrizedLine<double, 3> cylinderLine2(cylinderOrigin2, cylinderOrientation2);
		Vector3d cylinderCross = cylinderOrientation1.cross(cylinderOrientation2);

		//1. Parallel Case
		if (cylinderCross.norm() <= eps) {
			Vector3d pointOrigin1OnLine2 = cylinderLine2.projection(cylinderOrigin1);
			Vector3d origin2ToPoint1 = pointOrigin1OnLine2 - cylinderOrigin2;
			double distOriginToPoint = origin2ToPoint1.norm();
			double distOriginToLine = cylinderLine2.distance(cylinderOrigin1);
			if (distOriginToPoint <= (shape2->getLength() + shape1->getLength()) / 2) {
				distance = distOriginToLine - (shape1->getRadius() + shape2->getRadius());
			}
			else {
				Vector3d pointOrigin2OnLine1 = cylinderLine1.projection(cylinderOrigin2);
				distance = (cylinderOrigin1 - pointOrigin2OnLine1).norm() - (shape1->getLength() / 2 + shape2->getLength() / 2);
			}
			return distance;
		}

		Eigen::Matrix3d m;
		m.row(0) = cylinderOrientation1;
		m.row(1) = cylinderOrientation2;
		m.row(2) = (cylinderOrigin2 - cylinderOrigin1);

		//2. Intersecting Case
		if (fabs(m.determinant()) <= eps) {
			// find intersection of lines
			Vector3d pN = cylinderOrientation1.cross(cylinderOrientation2);
			pN.normalize();
			Vector3d pn2 = cylinderOrientation2.cross(pN);
			pn2.normalize();
			Hyperplane<double, 3> pCross2(pn2, cylinderOrigin2);
			Vector3d intersect = cylinderLine1.intersectionPoint(pCross2);
			// calc top of cylinder 1
			Vector3d origin1ToIntersect = intersect - cylinderOrigin1;
			Vector3d cylinderTop1 = cylinderOrigin1 + (shape1->getLength() / 2)*origin1ToIntersect.normalized();
			// calc top of cylinder 2
			Vector3d origin2ToIntersect = intersect - cylinderOrigin2;
			Vector3d cylinderTop2 = cylinderOrigin2 + (shape2->getLength() / 2)*origin2ToIntersect.normalized();
			// calc corner of cylinder 1
			ParametrizedLine<double, 3> parallelTo1(cylinderOrigin2, cylinderOrientation1);
			Vector3d top1ToParallelLine = parallelTo1.projection(cylinderTop1)-cylinderTop1;
			Vector3d cylinderCorner1 = cylinderTop1 + shape1->getRadius()*top1ToParallelLine.normalized();
			// calc corner of cylinder 2
			ParametrizedLine<double, 3> parallelTo2(cylinderOrigin1, cylinderOrientation2);
			Vector3d top2ToParallelLine = parallelTo2.projection(cylinderTop2) - cylinderTop2;
			Vector3d cylinderCorner2 = cylinderTop2 + shape2->getRadius()*top2ToParallelLine.normalized();
			// calc distances
			Vector3d corner1ProjectedToLine2 = cylinderLine2.projection(cylinderCorner1);
			Vector3d corner2ProjectedToLine1 = cylinderLine1.projection(cylinderCorner2);
			Vector3d corner1ToProjectionOnLine2 = corner1ProjectedToLine2 - cylinderCorner1;
			Vector3d corner2ToProjectionOnLine1 = corner2ProjectedToLine1 - cylinderCorner2;
			double distOrigin1ToProjectionOnLine1 = (cylinderOrigin1 - corner2ProjectedToLine1).norm();
			double distOrigin2ToProjectionOnLine2 = (cylinderOrigin2 - corner1ProjectedToLine2).norm();
			bool projection2InCylinder1 = distOrigin1ToProjectionOnLine1 <= (shape1->getLength() / 2);
			bool projection1InCylinder2 = distOrigin2ToProjectionOnLine2 <= (shape2->getLength() / 2);
			// corner of cylinder2 could touch hull of cylinder 1
			if (projection2InCylinder1 && !projection1InCylinder2) {
				distance = corner2ToProjectionOnLine1.norm() - shape1->getRadius();
			}
			// corner of cylinder1 could touch hull of cylinder 2
			else if (!projection2InCylinder1 && projection1InCylinder2) {
				distance = corner1ToProjectionOnLine2.norm() - shape2->getRadius();
			}
			// corner of cylinder1 could touch corner of cylinder 2
			else if (projection2InCylinder1 && projection1InCylinder2) {
				double corner2Corner = (cylinderCorner1 - cylinderCorner2).norm();
				double cornerToHull1 = corner2ToProjectionOnLine1.norm() - shape1->getRadius();
				double cornerToHull2 = corner1ToProjectionOnLine2.norm() - shape2->getRadius();
				distance = std::min(corner2Corner, std::min(cornerToHull1, cornerToHull2));
			}
			else {
				distance = (cylinderCorner1 - cylinderCorner2).norm();
			}
			return distance;
		}
		//3. Skewed Case
		// find the shortest distance connecting the two disjoint rotational cylinder axis
		Vector3d pN = cylinderOrientation1.cross(cylinderOrientation2);
		pN.normalize();
		Vector3d pN2 = cylinderOrientation1.cross(pN);
		pN2.normalize();
		Vector3d pN3 = cylinderOrientation2.cross(pN);
		pN3.normalize();
		Hyperplane<double, 3> pCross2(pN2, cylinderOrigin1);
		Vector3d cylinder2Intersect = cylinderLine2.intersectionPoint(pCross2);
		Hyperplane<double, 3> pCross3(pN3, cylinderOrigin2);
		Vector3d cylinder1Intersect = cylinderLine1.intersectionPoint(pCross3);
		// Find out if intersect points are in the cylinders
		Vector3d origin1ToIntersect1 = cylinder1Intersect - cylinderOrigin1;
		Vector3d origin2ToIntersect2 = cylinder2Intersect - cylinderOrigin2;
		//Case 3.1: If intersect points are both in cylinders -> distance between hulls
		if ((origin1ToIntersect1.norm() <= shape1->getLength() / 2) && (origin2ToIntersect2.norm() <= shape2->getLength() / 2)) {
			Vector3d intersect2ProjectedToLine1 = cylinderLine1.projection(cylinder2Intersect);
			Vector3d intersect1ProjectedToLine2 = cylinderLine2.projection(cylinder1Intersect);
			Vector3d verticalOnLine2 = (cylinder1Intersect - intersect1ProjectedToLine2).normalized();
			Vector3d verticalOnLine1 = (cylinder2Intersect - intersect2ProjectedToLine1).normalized();
			Vector3d intersect1ToIntersect2 = cylinder2Intersect - cylinder1Intersect;
			double angleOn1 = std::acos(verticalOnLine1.dot(intersect1ToIntersect2.normalized()));
			double angleOn2 = std::acos(verticalOnLine2.dot(-intersect1ToIntersect2.normalized()));
			double distanceToHull1 = shape1->getRadius() / std::cos(angleOn1);
			double distanceToHull2 = shape2->getRadius() / std::cos(angleOn2);
			distance = intersect1ToIntersect2.norm() - (distanceToHull1 + distanceToHull2);
		}
		//Case 3.2: If intersect points outside of either cylinder -> distance between hull and corner
		else {
			// calc top of cylinder 1
			Vector3d origin1ToIntersect = cylinder1Intersect - cylinderOrigin1;
			Vector3d cylinderTop1 = cylinderOrigin1 + (shape1->getLength() / 2)*origin1ToIntersect.normalized();
			// calc top of cylinder 2
			Vector3d origin2ToIntersect = cylinder2Intersect - cylinderOrigin2;
			Vector3d cylinderTop2 = cylinderOrigin2 + (shape2->getLength() / 2)*origin2ToIntersect.normalized();
			// calc corner of cylinder 1
			ParametrizedLine<double, 3> parallelTo1(cylinderOrigin2, cylinderOrientation1);
			Vector3d top1ToParallelLine = parallelTo1.projection(cylinderTop1) - cylinderTop1;
			Vector3d cylinderCorner1 = cylinderTop1 + shape1->getRadius()*top1ToParallelLine.normalized();
			// calc corner of cylinder 2
			ParametrizedLine<double, 3> parallelTo2(cylinderOrigin1, cylinderOrientation2);
			Vector3d top2ToParallelLine = parallelTo2.projection(cylinderTop2) - cylinderTop2;
			Vector3d cylinderCorner2 = cylinderTop2 + shape2->getRadius()*top2ToParallelLine.normalized();
			// calc distances
			Vector3d corner1ProjectedToLine2 = cylinderLine2.projection(cylinderCorner1);
			Vector3d corner2ProjectedToLine1 = cylinderLine1.projection(cylinderCorner2);
			Vector3d corner1ToProjectionOnLine2 = corner1ProjectedToLine2 - cylinderCorner1;
			Vector3d corner2ToProjectionOnLine1 = corner2ProjectedToLine1 - cylinderCorner2;
			double distOrigin1ToProjectionOnLine1 = (cylinderOrigin1 - corner2ProjectedToLine1).norm();
			double distOrigin2ToProjectionOnLine2 = (cylinderOrigin2 - corner1ProjectedToLine2).norm();
			bool projection2InCylinder1 = distOrigin1ToProjectionOnLine1 <= (shape1->getLength() / 2);
			bool projection1InCylinder2 = distOrigin2ToProjectionOnLine2 <= (shape2->getLength() / 2);
			// corner of cylinder2 could touch hull of cylinder 1
			if (projection2InCylinder1 && !projection1InCylinder2) {
				distance = corner2ToProjectionOnLine1.norm() - shape1->getRadius();
			}
			// corner of cylinder1 could touch hull of cylinder 2
			else if (!projection2InCylinder1 && projection1InCylinder2) {
				distance = corner1ToProjectionOnLine2.norm() - shape2->getRadius();
			}
			// corner of cylinder1 could touch corner of cylinder 2
			else if (projection2InCylinder1 && projection1InCylinder2) {
				double corner2Corner = (cylinderCorner1 - cylinderCorner2).norm();
				double cornerToHull1 = corner2ToProjectionOnLine1.norm() - shape1->getRadius();
				double cornerToHull2 = corner1ToProjectionOnLine2.norm() - shape2->getRadius();
				distance = std::min(corner2Corner, std::min(cornerToHull1, cornerToHull2));
			}
			else {
				distance = (cylinderCorner1 - cylinderCorner2).norm();
			}
		}
		
		return distance;
	}
}