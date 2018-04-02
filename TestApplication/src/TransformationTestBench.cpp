#include "..\include\TransformationTestBench.h"
#include "..\include\TestUtilities.h"

#include <iostream>
#include <MetaSpecification.h>
#include <CLibCollisionController.h>
#include <Shape.h>
#include <SimulationObject.h>
#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>

namespace tests {

	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using simobj::specs::CoordinateType;
	using simobj::Agent;
	using simobj::Site;
	using simobj::AgentCluster;
	using simobj::ReferenceFrame;
	using namespace clib;

	using Eigen::Vector3d;
	using utils::Quaternion;

	void TransformationTestBench::testAgentTransformations_setup() {
		//setup
		SiteSpecArray ssa;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));
		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("AT-Test", std::make_shared<CLibCollisionController>(meta)));
	}

	void TransformationTestBench::testAgentClusterTransformations_setup() {
		//setup
		SiteSpecArray ssa;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("ACT-Test", std::make_shared<CLibCollisionController>(meta)));
	}

	void TransformationTestBench::setup() {
		vector<ShapePtr> shapeArray;
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
		shapes.insert(std::make_pair("shapes1", shapeArray));

		testAgentTransformations_setup();
		testAgentClusterTransformations_setup();
	}

	void TransformationTestBench::testAgentTransformations() {
		CLibCollisionController& cc = *CLibController["AT-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);

		// test cases
		try {
			auto agnt = std::static_pointer_cast<Agent>(cc.getAgent(0));
			auto ste0 = std::static_pointer_cast<Site>(agnt->getSite(0));
			auto ste1 = std::static_pointer_cast<Site>(agnt->getSite(1));

			agnt->move(Vector3d(12, 0, 0));
			utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Global), "Agent translation test 1 - global frame");
			utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Local), "Agent translation test 2 - local frame");
			utils::compareCoordinates(Vector3d(17, 0, 0), ste0->getPosition(ReferenceFrame::Global), "Agent + site translation test 1 - global frame");
			utils::compareCoordinates(Vector3d(12, 5, 0), ste1->getPosition(ReferenceFrame::Global), "Agent + site translation test 2 - global frame");
			utils::compareCoordinates(Vector3d(5, 0, 0), ste0->getPosition(ReferenceFrame::Local), "Agent + site translation test 3 - local frame");
			utils::compareCoordinates(Vector3d(0, 5, 0), ste1->getPosition(ReferenceFrame::Local), "Agent + site translation test 4 - local frame");

			auto agnt2 = std::static_pointer_cast<Agent>(cc.getAgent(2));
			auto ste02 = std::static_pointer_cast<Site>(agnt2->getSite(0));
			auto ste12 = std::static_pointer_cast<Site>(agnt2->getSite(1));

			agnt2->move(Vector3d(-12, 0, 0));
			agnt2->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)));
			utils::compareOrientation(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)), agnt2->getOrientation(ReferenceFrame::Global), "Agent rotation test 1 - global frame");
			utils::compareOrientation(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)), agnt2->getOrientation(ReferenceFrame::Local), "Agent rotation test 2 - local frame");
			utils::compareCoordinates(Vector3d(-12, 0, 0), agnt2->getPosition(ReferenceFrame::Global), "Agent translation + rotation test 1 - global frame");
			utils::compareCoordinates(Vector3d(-12, 5, 0), ste02->getPosition(ReferenceFrame::Global), "Agent + site rotation test 1 - global frame");
			utils::compareCoordinates(Vector3d(-17, 0, 0), ste12->getPosition(ReferenceFrame::Global), "Agent + site rotation test 2 - global frame");
			utils::compareCoordinates(Vector3d(5, 0, 0), ste02->getPosition(ReferenceFrame::Local), "Agent + site rotation test 3 - local frame");
			utils::compareCoordinates(Vector3d(0, 5, 0), ste12->getPosition(ReferenceFrame::Local), "Agent + site rotation test 4 - local frame");

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		BOOST_TEST(cc.createAgentCluster(0, "default") == true);
		BOOST_TEST(cc.addAgentToCluster(0, 0) == true, "Add agent 0 to cluster 0");
		BOOST_TEST(cc.addAgentToCluster(1, 0) == true, "Add agent 1 to cluster 0");
		BOOST_TEST(cc.addAgentToCluster(2, 0) == true, "Add agent 2 to cluster 0");

		// test cases
		try {
			auto agnt = std::static_pointer_cast<Agent>(cc.getAgent(0));
			auto ste0 = std::static_pointer_cast<Site>(agnt->getSite(0));
			auto ste1 = std::static_pointer_cast<Site>(agnt->getSite(1));

			utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Global), "Agent translation test 3 - global frame");
			utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Local), "Agent translation test 4 - local frame");
			utils::compareCoordinates(Vector3d(17, 0, 0), ste0->getPosition(ReferenceFrame::Global), "Agent + site translation test 3 - global frame");
			utils::compareCoordinates(Vector3d(12, 5, 0), ste1->getPosition(ReferenceFrame::Global), "Agent + site translation test 4 - global frame");
			utils::compareCoordinates(Vector3d(5, 0, 0), ste0->getPosition(ReferenceFrame::Local), "Agent + site translation test 5 - local frame");
			utils::compareCoordinates(Vector3d(0, 5, 0), ste1->getPosition(ReferenceFrame::Local), "Agent + site translation test 6 - local frame");

			auto agnt2 = std::static_pointer_cast<Agent>(cc.getAgent(2));
			auto ste02 = std::static_pointer_cast<Site>(agnt2->getSite(0));
			auto ste12 = std::static_pointer_cast<Site>(agnt2->getSite(1));

			utils::compareOrientation(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)), agnt2->getOrientation(ReferenceFrame::Global), "Agent rotation test 3 - global frame");
			utils::compareOrientation(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)), agnt2->getOrientation(ReferenceFrame::Local), "Agent rotation test 4 - local frame");
			utils::compareCoordinates(Vector3d(-12, 0, 0), agnt2->getPosition(ReferenceFrame::Global), "Agent translation + rotation test 2 - global frame");
			utils::compareCoordinates(Vector3d(-12, 5, 0), ste02->getPosition(ReferenceFrame::Global), "Agent + site rotation test 5 - global frame");
			utils::compareCoordinates(Vector3d(-17, 0, 0), ste12->getPosition(ReferenceFrame::Global), "Agent + site rotation test 6 - global frame");
			utils::compareCoordinates(Vector3d(5, 0, 0), ste02->getPosition(ReferenceFrame::Local), "Agent + site rotation test 7 - local frame");
			utils::compareCoordinates(Vector3d(0, 5, 0), ste12->getPosition(ReferenceFrame::Local), "Agent + site rotation test 8 - local frame");

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}
		if (showVisualization) BOOST_TEST(cc.displayAgentCluster(0) == true);
	}

	void TransformationTestBench::testAgentClusterTransformations() {
		CLibCollisionController& cc = *CLibController["ACT-Test"];
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

		// test cases
		try {
			auto cluster = std::static_pointer_cast<AgentCluster>(cc.getAgentCluster(0));
			auto agnt = std::static_pointer_cast<Agent>(cc.getAgent(0));
			auto ste0 = std::static_pointer_cast<Site>(agnt->getSite(0));
			auto ste1 = std::static_pointer_cast<Site>(agnt->getSite(1));
			auto agnt2 = std::static_pointer_cast<Agent>(cc.getAgent(4));
			auto ste20 = std::static_pointer_cast<Site>(agnt->getSite(0));
			auto ste21 = std::static_pointer_cast<Site>(agnt->getSite(1));
			auto posAgnt2 = agnt2->getPosition(ReferenceFrame::Global);
			auto posSte20 = ste20->getPosition(ReferenceFrame::Global);
			auto posSte21 = ste21->getPosition(ReferenceFrame::Global);

			cluster->setPosition(Vector3d(5, 5, 5));
			utils::compareCoordinates(Vector3d(5, 5, 5), agnt->getPosition(ReferenceFrame::Global), "Cluster translation test 1 - agent0, global");
			utils::compareCoordinates(Vector3d(10, 5, 5), ste0->getPosition(ReferenceFrame::Global), "Cluster translation test 1 - site0, global");
			cluster->rotate(Quaternion::FromTwoVectors(Vector3d(0, 0, 1), Vector3d(0, 1, 0)));
			utils::compareCoordinates(Vector3d(5, 5, 5), agnt->getPosition(ReferenceFrame::Global), "Cluster rotation test 1 - agent0, global");
			utils::compareCoordinates(Vector3d(10, 5, 5), ste0->getPosition(ReferenceFrame::Global), "Cluster rotation test 1 - site0, global");
			cluster->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 0, 1)));
			utils::compareCoordinates(Vector3d(5, 5, 5), agnt->getPosition(ReferenceFrame::Global), "Cluster rotation test 2 - agent0, global");
			utils::compareCoordinates(Vector3d(5, 10, 5), ste0->getPosition(ReferenceFrame::Global), "Cluster rotation test 2 - site0, global");
			cluster->rotate(Quaternion::FromTwoVectors(Vector3d(0, 1, 0), Vector3d(0, 0, 1)));
			cluster->rotate(Quaternion::FromTwoVectors(Vector3d(0, 1, 0), Vector3d(1, 0, 0)));
			cluster->move(Vector3d(-5, -5, -5));
			utils::compareCoordinates(Vector3d(0, 0, 0), agnt->getPosition(ReferenceFrame::Global), "Cluster rotation + translation test 1 - agent0, global");
			utils::compareCoordinates(Vector3d(5, 0, 0), ste0->getPosition(ReferenceFrame::Global), "Cluster rotation + translation test 1 - site0, global");
			//sanity check
			utils::compareCoordinates(posSte21, ste21->getPosition(ReferenceFrame::Global), "Cluster rotation + translation test 2 - site41, global");

		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		if (showVisualization) BOOST_TEST(cc.displayAgentCluster(0) == true);
	}


	void TransformationTestBench::runAllTests() {
		testAgentTransformations();
		testAgentClusterTransformations();
	}

}