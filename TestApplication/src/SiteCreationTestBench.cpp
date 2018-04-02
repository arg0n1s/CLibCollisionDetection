#include "..\include\SiteCreationTestBench.h"
#include "..\include\TestUtilities.h"

#include <iostream>
#include <MetaSpecification.h>
#include <CLibCollisionController.h>
#include <Shape.h>
#include <SimulationObject.h>
#include <Agent.h>

namespace tests {

	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using simobj::specs::CoordinateType;
	using simobj::Agent;
	using namespace clib;

	using Eigen::Vector3d;

	void SiteCreationTestBench::siteThroughKarthesianAbsoluteTest_setup() {
		// setup
		SiteSpecArray ssa, ssa2, ssa3;
		ssa.push_back(clib::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute));
		ssa.push_back(clib::createSiteSpecification(1, 0, 5, 0, CoordinateType::KarthesianAbsolute));

		ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 3, CoordinateType::KarthesianAbsolute));
		ssa2.push_back(clib::createSiteSpecification(1, 2, 0, 0, CoordinateType::KarthesianAbsolute));

		ssa3.push_back(clib::createSiteSpecification(0, 2, 0, 0, CoordinateType::KarthesianAbsolute));
		ssa3.push_back(clib::createSiteSpecification(1, 0, 3, 0, CoordinateType::KarthesianAbsolute));
		ssa3.push_back(clib::createSiteSpecification(2, 0, 0, 1, CoordinateType::KarthesianAbsolute));

		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		asa.push_back(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ssa2));
		asa.push_back(clib::createAgentSpecification("EllipsoidAgent", shapes["shapes1"][2], ssa3));

		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("STKAT-Test",std::make_shared<CLibCollisionController>(meta)));
	}

	void SiteCreationTestBench::siteThroughKarthHullPointerTest_setup() {
		// setup
		SiteSpecArray ssa, ssa2, ssa3;
		ssa.push_back(clib::createSiteSpecification(0, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 0, 1, 0, CoordinateType::KarthesianPointerToHull));

		ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 1, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(1, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(2, 1, 1, 1, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(3, -1, -1, -1, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(4, 0, 1, 2, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(5, 0, 1, 1.5, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(6, 1, 0, 1, CoordinateType::KarthesianPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(7, 0, 1, 1, CoordinateType::KarthesianPointerToHull));

		ssa3.push_back(clib::createSiteSpecification(0, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
		ssa3.push_back(clib::createSiteSpecification(1, 0, 1, 0, CoordinateType::KarthesianPointerToHull));
		ssa3.push_back(clib::createSiteSpecification(2, 0, 0, 1, CoordinateType::KarthesianPointerToHull));

		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		asa.push_back(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ssa2));
		asa.push_back(clib::createAgentSpecification("EllipsoidAgent", shapes["shapes1"][2], ssa3));

		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("STKHP-Test", std::make_shared<CLibCollisionController>(meta)));
	}

	void SiteCreationTestBench::siteThroughParametrizedAbsoluteTest_setup() {
		// setup
		SiteSpecArray ssa, ssa2, ssa3;
		ssa.push_back(clib::createSiteSpecification(0, 5, M_PI / 2, 0, CoordinateType::ParametricAbsolute));
		ssa.push_back(clib::createSiteSpecification(1, 5, M_PI / 2, M_PI / 2, CoordinateType::ParametricAbsolute));

		ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 3, CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(1, 2, 0, 0, CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(2, 2, M_PI / 4, std::sqrt(2), CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(3, 2, M_PI * 5.0 / 4.0, -std::sqrt(2), CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(4, 1.5, M_PI / 2, 3, CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(5, 2, M_PI / 2, 3, CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(6, 2, 0, 2, CoordinateType::ParametricAbsolute));
		ssa2.push_back(clib::createSiteSpecification(7, 2, M_PI / 2, 2, CoordinateType::ParametricAbsolute));

		ssa3.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricAbsolute));
		ssa3.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricAbsolute));
		ssa3.push_back(clib::createSiteSpecification(2, 1, 0, 0, CoordinateType::ParametricAbsolute));

		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		asa.push_back(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ssa2));
		asa.push_back(clib::createAgentSpecification("EllipsoidAgent", shapes["shapes1"][2], ssa3));

		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("STPA-Test", std::make_shared<CLibCollisionController>(meta)));
	}

	void SiteCreationTestBench::siteThroughParamHullPointerTest_setup() {
		// setup
		SiteSpecArray ssa, ssa2, ssa3;
		ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));

		ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 3, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(1, 2, 0, 0, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(2, 2, M_PI / 4, std::sqrt(2), CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(3, 2, M_PI * 5.0 / 4.0, -std::sqrt(2), CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(4, 1.5, M_PI / 2, 3, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(5, 2, M_PI / 2, 3, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(6, 2, 0, 2, CoordinateType::ParametricPointerToHull));
		ssa2.push_back(clib::createSiteSpecification(7, 2, M_PI / 2, 2, CoordinateType::ParametricPointerToHull));

		ssa3.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
		ssa3.push_back(clib::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));
		ssa3.push_back(clib::createSiteSpecification(2, 1, 0, 0, CoordinateType::ParametricPointerToHull));

		AgentSpecArray asa;
		asa.push_back(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ssa));
		asa.push_back(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ssa2));
		asa.push_back(clib::createAgentSpecification("EllipsoidAgent", shapes["shapes1"][2], ssa3));

		MetaSpecification meta = clib::createMetaSpecification(asa);
		CLibController.insert(std::make_pair("STPHP-Test", std::make_shared<CLibCollisionController>(meta)));
	}

	void SiteCreationTestBench::setup() {
		vector<ShapePtr> shapeArray;
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));
		shapes.insert(std::make_pair("shapes1", shapeArray));

		siteThroughKarthesianAbsoluteTest_setup();
		siteThroughKarthHullPointerTest_setup();
		siteThroughParametrizedAbsoluteTest_setup();
		siteThroughParamHullPointerTest_setup();
	}

	void SiteCreationTestBench::siteThroughKarthesianAbsoluteTest() {
		CLibCollisionController& cc = *CLibController["STKAT-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(2, "EllipsoidAgent") == true);

		// test cases
		try {
			utils::compareCoordinates(Vector3d(5, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(0)->getPosition(), "Site1 through Karth. Abs. on Sphere - Test");
			utils::compareCoordinates(Vector3d(0, 5, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(1)->getPosition(), "Site2 through Karth. Abs. on Sphere - Test");

			utils::compareCoordinates(Vector3d(0, 0, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(0)->getPosition(), "Site1 through Karth. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(1)->getPosition(), "Site2 through Karth. Abs. on Cylinder - Test");

			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(0)->getPosition(), "Site1 through Karth. Abs. on Ellpsoid - Test");
			utils::compareCoordinates(Vector3d(0, 3, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(1)->getPosition(), "Site2 through Karth. Abs. on Ellpsoid - Test");
			utils::compareCoordinates(Vector3d(0, 0, 1), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(2)->getPosition(), "Site3 through Karth. Abs. on Ellpsoid - Test");
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		// display results
		if (showVisualization) {
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}
	}

	void SiteCreationTestBench::siteThroughKarthHullPointerTest() {
		CLibCollisionController& cc = *CLibController["STKHP-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(2, "EllipsoidAgent") == true);

		// test cases
		try {
			utils::compareCoordinates(Vector3d(5, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(0)->getPosition(), "Site1 through Karth. Hull. ptr. on Sphere - Test");
			utils::compareCoordinates(Vector3d(0, 5, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(1)->getPosition(), "Site2 through Karth. Hull. ptr. on Sphere - Test");

			utils::compareCoordinates(Vector3d(0, 0, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(0)->getPosition(), "Site1 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(1)->getPosition(), "Site2 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(std::sqrt(2), std::sqrt(2), std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(2)->getPosition(), "Site3 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(-std::sqrt(2), -std::sqrt(2), -std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(3)->getPosition(), "Site4 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 1.5, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(4)->getPosition(), "Site5 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(5)->getPosition(), "Site6 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(6)->getPosition(), "Site7 through Karth. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(7)->getPosition(), "Site8 through Karth. Hull. ptr. on Cylinder - Test");

			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(0)->getPosition(), "Site1 through Karth. Hull. ptr. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 3, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(1)->getPosition(), "Site2 through Karth. Hull. ptr. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 0, 1), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(2)->getPosition(), "Site3 through Karth. Hull. ptr. on Ellipsoid - Test");
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		// display results
		if (showVisualization) {
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}
	}

	void SiteCreationTestBench::siteThroughParametrizedAbsoluteTest() {
		CLibCollisionController& cc = *CLibController["STPA-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(2, "EllipsoidAgent") == true);

		// test cases
		try {
			utils::compareCoordinates(Vector3d(5, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(0)->getPosition(), "Site1 through Param. Abs. on Sphere - Test");
			utils::compareCoordinates(Vector3d(0, 5, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(1)->getPosition(), "Site2 through Param. Abs. on Sphere - Test");

			utils::compareCoordinates(Vector3d(0, 0, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(0)->getPosition(), "Site1 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(1)->getPosition(), "Site2 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(std::sqrt(2), std::sqrt(2), std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(2)->getPosition(), "Site3 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(-std::sqrt(2), -std::sqrt(2), -std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(3)->getPosition(), "Site4 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 1.5, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(4)->getPosition(), "Site5 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(5)->getPosition(), "Site6 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(6)->getPosition(), "Site7 through Param. Abs. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(7)->getPosition(), "Site8 through Param. Abs. on Cylinder - Test");

			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(0)->getPosition(), "Site1 through Param. Abs. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 3, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(1)->getPosition(), "Site2 through Param. Abs. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 0, 1), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(2)->getPosition(), "Site3 through Param. Abs. on Ellipsoid - Test");
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		// display results
		if (showVisualization) {
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}
	}

	void SiteCreationTestBench::siteThroughParamHullPointerTest() {
		CLibCollisionController& cc = *CLibController["STPHP-Test"];
		BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
		BOOST_TEST(cc.createAgent(1, "CylinderAgent") == true);
		BOOST_TEST(cc.createAgent(2, "EllipsoidAgent") == true);

		// test cases
		try {
			utils::compareCoordinates(Vector3d(5, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(0)->getPosition(), "Site1 through Param. Hull. ptr. on Sphere - Test");
			utils::compareCoordinates(Vector3d(0, 5, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(1)->getPosition(), "Site2 through Param. Hull. ptr. on Sphere - Test");

			utils::compareCoordinates(Vector3d(0, 0, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(0)->getPosition(), "Site1 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(1)->getPosition(), "Site2 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(std::sqrt(2), std::sqrt(2), std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(2)->getPosition(), "Site3 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(-std::sqrt(2), -std::sqrt(2), -std::sqrt(2)), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(3)->getPosition(), "Site4 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 1.5, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(4)->getPosition(), "Site5 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 3), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(5)->getPosition(), "Site6 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(2, 0, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(6)->getPosition(), "Site7 through Param. Hull. ptr. on Cylinder - Test");
			utils::compareCoordinates(Vector3d(0, 2, 2), std::static_pointer_cast<simobj::Agent>(cc.getAgent(1))->getSite(7)->getPosition(), "Site8 through Param. Hull. ptr. on Cylinder - Test");

			utils::compareCoordinates(Vector3d(2, 0, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(0)->getPosition(), "Site1 through Param. Hull. ptr. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 3, 0), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(1)->getPosition(), "Site2 through Param. Hull. ptr. on Ellipsoid - Test");
			utils::compareCoordinates(Vector3d(0, 0, 1), std::static_pointer_cast<simobj::Agent>(cc.getAgent(2))->getSite(2)->getPosition(), "Site3 through Param. Hull. ptr. on Ellipsoid - Test");
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

		// display results
		if (showVisualization) {
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}
	}

	void SiteCreationTestBench::runAllTests() {
		siteThroughKarthesianAbsoluteTest();
		siteThroughKarthHullPointerTest();
		siteThroughParametrizedAbsoluteTest();
		siteThroughParamHullPointerTest();
	}

}