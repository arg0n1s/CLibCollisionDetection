#include "..\include\OctTreeTestBench.h"
#include "..\include\TestUtilities.h"

#include <iostream>
#include <MetaSpecification.h>
#include <CLibCollisionController.h>
#include <Shape.h>
#include <SimulationObject.h>
#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <OctTree.h>
#include <OctTreeNode.h>

namespace tests {

	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using simobj::specs::CoordinateType;
	using simobj::Agent;
	using simobj::Site;
	using simobj::AgentCluster;
	using simobj::ReferenceFrame;
	using collision::CollisionDetection;
	using collision::octtree::OctTree;
	using collision::octtree::OctTreeNode;
	using collision::octtree::TreePtr;
	using collision::octtree::NodePtr;
	using collision::octtree::IdSet;

	using namespace clib;

	using Eigen::Vector3d;
	using utils::Quaternion;

	void OctTreeTestBench::testOctTreeConstrution_setup() {
		//setup
		SiteSpecArray ssa;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("Contruction-Test", std::make_shared<CLibCollisionController>(meta)));
		CLibController["Contruction-Test"]->setInitialRootDiameter(4.0);
		CLibController["Contruction-Test"]->setMinimalLeafDiameter(2.0);
	}

	void OctTreeTestBench::testOctTreeNearestSearch_setup() {
		SiteSpecArray ssa;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("Search-Test", std::make_shared<CLibCollisionController>(meta)));
		CLibController["Search-Test"]->setInitialRootDiameter(4.0);
		CLibController["Search-Test"]->setMinimalLeafDiameter(2.0);
	}

	void OctTreeTestBench::testOctTreeCollision_setup() {
		SiteSpecArray ssa;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
		SiteSpecArray ssa2;
		ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 1, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(2, 0, 0, -1, CoordinateType::ParametricPointerToHull));
		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		asa.push_back(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ssa2));
		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("Collision-Test", std::make_shared<CLibCollisionController>(meta)));
		CLibController["Collision-Test"]->setInitialRootDiameter(4.0);
		CLibController["Collision-Test"]->setMinimalLeafDiameter(2.0);
	}

	void OctTreeTestBench::setup() {
		vector<ShapePtr> shapeArray;
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 1.0));
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 1.0, 6.0));
		shapes.insert(std::make_pair("shapes1", shapeArray));

		testOctTreeConstrution_setup();
		testOctTreeNearestSearch_setup();
		testOctTreeCollision_setup();
	}

	void OctTreeTestBench::testOctTreeConstrution() {
		CLibCollisionController& cc = *CLibController["Contruction-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(3, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(4, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(5, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(6, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(7, "SphereAgent") == true);

		// test cases
		try {
			cc.connectAgents(0, 1, 0, 1);
			cc.connectAgents(1, 2, 0, 1);
			cc.connectAgents(2, 3, 0, 1);
			cc.connectAgents(3, 4, 0, 1);
			cc.connectAgents(4, 5, 0, 1);
			cc.connectAgents(5, 6, 0, 1);
			cc.connectAgents(6, 7, 0, 1);

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}
		BOOST_TEST(cc.addAgentClusterToCollisionDetector(0) == true);
		if (showVisualization) BOOST_TEST(cc.displayClusterCollisionTree(0) == true);
	}

	void OctTreeTestBench::testOctTreeNearestSearch() {
		CLibCollisionController& cc = *CLibController["Search-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(3, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(4, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(5, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(6, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(7, "SphereAgent") == true);

		try {
			cc.connectAgents(0, 1, 0, 1);
			cc.connectAgents(1, 2, 0, 1);
			cc.connectAgents(2, 3, 0, 1);
			cc.connectAgents(3, 4, 0, 1);
			cc.connectAgents(4, 5, 0, 1);
			cc.connectAgents(5, 6, 0, 1);
			cc.connectAgents(6, 7, 0, 1);

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		BOOST_TEST(cc.addAgentClusterToCollisionDetector(0) == true);
		auto& cd = cc.getCollisionDetector();
		auto tree = cd.getTree(0);
		// Test1
		auto node = tree->getNearest(1, 1, 0);
		auto ids = node->getIds();
		IdSet<unsigned int> requiredIds = { 0,1 };
		utils::checkRequiredIds(ids, requiredIds, "\"Find nearest Node - Test1\"");
		// Test2
		node = tree->getNearest(1, 1, 10);
		ids = node->getIds();
		requiredIds = { 0,1 };
		utils::checkRequiredIds(ids, requiredIds, "\"Find nearest Node - Test2\"");
		// Test3
		node = tree->getNearest(5, 1, 1);
		ids = node->getIds();
		requiredIds = { 2 };
		utils::checkRequiredIds(ids, requiredIds, "\"Find nearest Node - Test3\"");
		// Test4
		node = tree->getNearest(5, 1, -1);
		ids = node->getIds();
		requiredIds = { 2 };
		utils::checkRequiredIds(ids, requiredIds, "\"Find nearest Node - Test4\"");
		// Test5
		node = tree->getNearest(1, 10, 3);
		ids = node->getIds();
		requiredIds = { 4,5 };
		utils::checkRequiredIds(ids, requiredIds, "\"Find nearest Node - Test5\"");

		if (showVisualization) BOOST_TEST(cc.displayClusterCollisionTree(0) == true);
	}

	void OctTreeTestBench::testOctTreeCollision() {
		CLibCollisionController& cc = *CLibController["Collision-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(3, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(4, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(5, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(6, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(7, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(8, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(9, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(10, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(11, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(12, "CylinderAgent") == true);

		try {
			cc.connectAgents(0, 1, 0, 1);
			cc.connectAgents(1, 2, 0, 1);
			cc.connectAgents(2, 3, 0, 1);
			cc.connectAgents(3, 4, 0, 1);
			cc.connectAgents(4, 5, 0, 1);
			cc.connectAgents(5, 6, 0, 1);
			cc.connectAgents(6, 7, 0, 1);

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		BOOST_TEST(cc.addAgentClusterToCollisionDetector(0) == true);
		auto& cd = cc.getCollisionDetector();
		auto cluster = std::static_pointer_cast<AgentCluster>(cc.getAgentCluster(0));
		auto candidate = std::static_pointer_cast<Agent>(cc.getAgent(8));
		SimObjPtr nearest = nullptr;
		double nearestDistance = 0;
		IdSet<unsigned int> ignoreIDs;
		// Test1
		candidate->setPosition(Vector3d(1, 1, 0));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 1.1 Collision expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 0) == true, "\n Error in Collision Test 1.1 Expexted candidate " << 0 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == (std::sqrt(2) - 2), "\n Error in Collision Test 1.1 Expexted distance " << (std::sqrt(2) - 2) << "but received " << nearestDistance);
		ignoreIDs = { 0 };
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 1.2 Collision expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 1) == true, "\n Error in Collision Test 1.2 Expexted candidate " << 1 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == (std::sqrt(2) - 2), "\n Error in Collision Test 1.2 Expexted distance " << (std::sqrt(2) - 2) << "but received " << nearestDistance);
		ignoreIDs = { 0,1 };
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 1.3 Collision not expected, but collision occured!");
		BOOST_TEST((nearest->getId() == 4) == true, "\n Error in Collision Test 1.3 Expexted candidate " << 4 << " but received " << nearest->getId());
		candidate->setPosition(Vector3d(1, 1, 6));
		ignoreIDs = {};
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 1.4 Collision not expected, but collision occured!");
		BOOST_TEST((nearest->getId() == 0) == true, "\n Error in Collision Test 1.4 Expexted candidate " << 0 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == (std::sqrt(38) - 2), "\n Error in Collision Test 1.4 Expexted distance " << (std::sqrt(38) - 2) << "but received " << nearestDistance);
		// Test2
		candidate = std::static_pointer_cast<Agent>(cc.getAgent(9));
		ignoreIDs = {};
		candidate->setPosition(Vector3d(1, 1, 0));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 2.1 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 0) == true, "\n Error in Collision Test 2.1 Expexted candidate " << 0 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == (std::sqrt(2) - 2), "\n Error in Collision Test 2.1 Expexted distance " << (std::sqrt(2) - 2) << "but received " << nearestDistance);
		candidate->setPosition(Vector3d(1, 1, 3.2));
		ignoreIDs = { 0 };
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 2.2 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 1) == true, "\n Error in Collision Test 2.2 Expexted candidate " << 1 << " but received " << nearest->getId());
		candidate->setPosition(Vector3d(1, 1, 4.0));
		ignoreIDs = {};
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 2.3 Collision not expected, but collision occured!");
		BOOST_TEST((nearest->getId() == 0) == true, "\n Error in Collision Test 2.3 Expexted candidate " << 0 << " but received " << nearest->getId());
		candidate->setPosition(Vector3d(3, 1, 1.0));
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, 1, 1.2)));
		ignoreIDs = { 1 };
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 2.4 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 2) == true, "\n Error in Collision Test 2.4 Expexted candidate " << 2 << " but received " << nearest->getId());
		// Test3
		candidate->setPosition(Vector3d(0, 0, 0));
		candidate->setOrientation(Quaternion::Identity());
		try {
			cc.connectAgents(10, 11, 1, 0);
			cc.connectAgents(11, 12, 2, 1);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}
		BOOST_TEST(cc.addAgentClusterToCollisionDetector(1) == true);
		cluster = std::static_pointer_cast<AgentCluster>(cc.getAgentCluster(1));
		bool collision = false;

		// Parallel cases
		ignoreIDs = {};
		candidate->setPosition(Vector3d(3, 0, -2));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.1.1 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.1.1 Expexted candidate " << 10 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == 1, "\n Error in Collision Test 3.1.1 Expexted distance " << 1 << "but received " << nearestDistance);

		ignoreIDs = {};
		candidate->setPosition(Vector3d(1.5, 0, 1));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.1.2 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.1.2 Expexted candidate " << 10 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == -0.5, "\n Error in Collision Test 3.1.2 Expexted distance " << -0.5 << "but received " << nearestDistance);

		ignoreIDs = {};
		candidate->setPosition(Vector3d(1.5, 0, 6.2));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.1.3 Collision was not expected, but collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.1.3 Expexted candidate " << 10 << " but received " << nearest->getId());
		BOOST_TEST(nearestDistance == 0.2, "\n Error in Collision Test 3.1.2 Expexted distance " << 0.2 << "but received " << nearestDistance);

		// Intersecting Cases
		ignoreIDs = {};
		candidate->setPosition(Vector3d(0, -3, 0));
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, 1, 1)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.2.1 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.2.1 Expexted candidate " << 10 << " but received " << nearest->getId());

		candidate->setPosition(Vector3d(0, -3, 2));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.2.2 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.2.2 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(0, -2, 4));
		candidate->setOrientation(Quaternion::Identity());
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, 1, 0)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.2.3 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.2.3 Expexted candidate " << 10 << " but received " << nearest->getId());

		candidate->setPosition(Vector3d(0, -2.5, 3.5));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.2.4 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.2.4 Expexted candidate " << 10 << "but received " << nearest->getId());

		// Skewed Cases
		ignoreIDs = {};
		candidate->setPosition(Vector3d(3, 0, 2));
		candidate->setOrientation(Quaternion::Identity());
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0.9, 1, 1)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.3.1 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.1 Expexted candidate " << 10 << " but received " << nearest->getId());
		ignoreIDs = {};

		ignoreIDs = {};
		candidate->setPosition(Vector3d(1.9, 0, 2));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.3.2 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.2 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(3, -2, 1));
		candidate->setOrientation(Quaternion::Identity());
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, 1, 1)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.3.3 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.3 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(-1.9, -2, 1));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.3.4 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.4 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(0.5, -2, 6.5));
		candidate->setOrientation(Quaternion::Identity());
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, -1, 1)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.3.5 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.5 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(0.5, -2, 5));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.3.6 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.6 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(0.5, -2, 6.5));
		candidate->setOrientation(Quaternion::Identity());
		candidate->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0.9, -1, 1)));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == false, "\n Error in Collision Test 3.3.7 Collision was not expected, but a collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.7 Expexted candidate " << 10 << " but received " << nearest->getId());

		ignoreIDs = {};
		candidate->setPosition(Vector3d(0.5, -2, 5));
		BOOST_TEST(cd.checkForCollision(cluster, ignoreIDs, candidate, nearest, nearestDistance) == true, "\n Error in Collision Test 3.3.8 Collision was expected, but no collision occured!");
		BOOST_TEST((nearest->getId() == 10) == true, "\n Error in Collision Test 3.3.8 Expexted candidate " << 10 << " but received " << nearest->getId());
	}

	void OctTreeTestBench::runAllTests() {
		testOctTreeConstrution();
		testOctTreeNearestSearch();
		testOctTreeCollision();
	}
}