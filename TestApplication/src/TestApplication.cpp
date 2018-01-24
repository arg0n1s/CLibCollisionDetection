#pragma once
#define BOOST_TEST_MODULE TestApplication

#include <boost/test/included/unit_test.hpp>
#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <CLibCollisionController.h>
#include <..\include\OctTree.h>

namespace simobj {
	namespace tests {

		using Eigen::Vector3d;
		using Quaternion = Eigen::Quaternion<double>;
		using simobj::shapes::ShapePtr;
		using simobj::specs::MetaSpecification;
		using simobj::specs::AgentSpecification;
		using simobj::specs::SiteSpecification;
		using SiteSpecArray = std::vector<SiteSpecification>;
		using AgentSpecArray = std::vector<AgentSpecification>;
		using ShapesArray = std::vector<ShapePtr>;
		using clib::CLibCollisionController;
		using simobj::specs::CoordinateType;
		using simobj::shapes::ShapeType;

		namespace utils {
			static const double EPS = 0.00001;

			void compareCoordinates(const Vector3d& correct, const Vector3d& given, const std::string& testname) {
				BOOST_TEST(correct.x() == given.x(), "\n Error for x-coordinates in: " << testname << ". Given x value was: " << given.x() << ", but should have been: " << correct.x() << ".");
				BOOST_TEST(correct.y() == given.y(), "\n Error for y-coordinates in: " << testname << ". Given y value was: " << given.y() << ", but should have been: " << correct.y() << ".");
				BOOST_TEST(correct.z() == given.z(), "\n Error for z-coordinates in: " << testname << ". Given z value was: " << given.z() << ", but should have been: " << correct.z() << ".");
			}

			void compareOrientation(const Quaternion& correct, const Quaternion& given, const std::string& testname) {
				BOOST_TEST(correct.x() == given.x(), "\n Error for x-value in: " << testname << ". Given x value was: " << given.x() << ", but should have been: " << correct.x() << ".");
				BOOST_TEST(correct.y() == given.y(), "\n Error for y-value in: " << testname << ". Given y value was: " << given.y() << ", but should have been: " << correct.y() << ".");
				BOOST_TEST(correct.z() == given.z(), "\n Error for z-value in: " << testname << ". Given z value was: " << given.z() << ", but should have been: " << correct.z() << ".");
				BOOST_TEST(correct.w() == given.w(), "\n Error for w-value in: " << testname << ". Given w value was: " << given.w() << ", but should have been: " << correct.w() << ".");
			}

		}

		void testExceptions() {
			bool shapeTest1, shapeTest2, shapeTest3, shapeTest4, shapeTest5, shapeTest6;
			bool agentSpecTest1;
			bool metaSpecTest1;
			bool simulationContainerTest1, simulationContainerTest2, simulationContainerTest3, simulationContainerTest4, simulationContainerTest5;
			// setup1
			SiteSpecArray ssa, ssa2, ssa3, ssa4, ssa5, ssa6;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 3, CoordinateType::KarthesianAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 2, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa4.push_back(CLibCollisionController::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa4.push_back(CLibCollisionController::createSiteSpecification(0, 5, M_PI/2, 0, CoordinateType::KarthesianAbsolute));
			ssa5.push_back(CLibCollisionController::createSiteSpecification(0, 5, M_PI / 2, 0, static_cast<CoordinateType>(4)));
			ssa6.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 0, CoordinateType::KarthesianPointerToHull));

			shapeTest1 = shapeTest2 = shapeTest3 = shapeTest4 = shapeTest5 = shapeTest6 = false;
			ShapesArray sa;
			// trigger exceptions
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 0.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest1 = true;
			}
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 0.0, 1.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest2 = true;
			}
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 0.0, 1.0, 1.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest3 = true;
			}
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 0.0, 0.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest4 = true;
			}
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 0.0, 1.0, 1.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest5 = true;
			}
			try {
				sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 0.0));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				shapeTest6 = true;
			}
			// setup2
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));

			AgentSpecArray asa, asa2, asa3;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			agentSpecTest1 = false;
			//trigger exceptions
			try {
				asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent1", sa[0], ssa4));
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				agentSpecTest1 = true;
			}
			// setup3
			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);

			metaSpecTest1 = false;
			//trigger exceptions
			try {
				asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
				MetaSpecification meta2 = CLibCollisionController::createMetaSpecification(asa);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				metaSpecTest1 = true;
			}
			BOOST_TEST(shapeTest1 == true, "ShapeSpec test 1 did not trigger the expected exception!");
			BOOST_TEST(shapeTest2 == true, "ShapeSpec test 2 did not trigger the expected exception!");
			BOOST_TEST(shapeTest3 == true, "ShapeSpec test 3 did not trigger the expected exception!");
			BOOST_TEST(shapeTest4 == true, "ShapeSpec test 4 did not trigger the expected exception!");
			BOOST_TEST(shapeTest5 == true, "ShapeSpec test 5 did not trigger the expected exception!");
			BOOST_TEST(shapeTest6 == true, "ShapeSpec test 6 did not trigger the expected exception!");
			BOOST_TEST(agentSpecTest1 == true, "AgentSpec test 1 did not trigger the expected exception!");
			BOOST_TEST(metaSpecTest1 == true, "MetaSpec test 1 did not trigger the expected exception!");
			// setup4
			CLibCollisionController cc(meta);
			cc.createAgent(0, "SphereAgent");
			cc.createAgent(1, "CylinderAgent");
			cc.createAgent(2, "EllipsoidAgent");

			asa2.push_back(CLibCollisionController::createAgentSpecification("SphereAgent1", sa[0], ssa5));
			MetaSpecification meta2 = CLibCollisionController::createMetaSpecification(asa2);
			CLibCollisionController cc2(meta2);

			asa3.push_back(CLibCollisionController::createAgentSpecification("SphereAgent2", sa[0], ssa6));
			MetaSpecification meta3 = CLibCollisionController::createMetaSpecification(asa3);
			CLibCollisionController cc3(meta3);
			

			simulationContainerTest1 = simulationContainerTest2 = simulationContainerTest3 = simulationContainerTest4 = simulationContainerTest5 = false;
			//trigger exceptions
			BOOST_TEST(cc.createAgent(0, "SphereAgent") == false, "Simulation container test 1 did not trigger the expected exception!");
			BOOST_TEST(cc.createAgent(3, "SphereAgent0") == false, "Simulation container test 2 did not trigger the expected exception!");
			BOOST_TEST(cc2.createAgent(4, "SphereAgent1") == false, "Simulation container test 3 did not trigger the expected exception!");
			BOOST_TEST(cc3.createAgent(5, "SphereAgent2") == false, "Simulation container test 4 did not trigger the expected exception!");
			try {
				cc.getAgent(4);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				simulationContainerTest1 = true;
			}
			try {
				std::static_pointer_cast<simobj::Agent>(cc.getAgent(0))->getSite(5);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				simulationContainerTest2 = true;
			}
			BOOST_TEST(simulationContainerTest1 == true, "Simulation container test 5 did not trigger the expected exception!");
			BOOST_TEST(simulationContainerTest2 == true, "Simulation container test 6 did not trigger the expected exception!");
			BOOST_TEST(cc.displayAgent(6) == false, "Simulation container test 7 did not trigger the expected exception!");
			try {
				cc.connectAgents(3, 2, 0, 0);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				simulationContainerTest3 = true;
			}
			try {
				cc.connectAgents(1, 2, 5, 0);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				simulationContainerTest4 = true;
			}
			try {
				cc.connectAgents(1, 1, 0, 0);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				simulationContainerTest5 = true;
			}
			BOOST_TEST(simulationContainerTest3 == true, "Simulation container test 8 did not trigger the expected exception!");
			BOOST_TEST(simulationContainerTest4 == true, "Simulation container test 9 did not trigger the expected exception!");
			BOOST_TEST(simulationContainerTest5 == true, "Simulation container test 10 did not trigger the expected exception!");
			
		}

		void siteThroughKarthesianAbsolute() {
			// setup
			SiteSpecArray ssa, ssa2, ssa3;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 0, 5, 0, CoordinateType::KarthesianAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 3, CoordinateType::KarthesianAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(1, 2, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 2, 0, 0, CoordinateType::KarthesianAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(1, 0, 3, 0, CoordinateType::KarthesianAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(2, 0, 0, 1, CoordinateType::KarthesianAbsolute));

			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);

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
			} catch (std::exception& e) {
				std::cout << e.what() << std::endl;
			}

			// display results
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}

		void siteThroughKarthHullPointer() {
			// setup
			SiteSpecArray ssa, ssa2, ssa3;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 0, 1, 0, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 1, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(1, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(2, 1, 1, 1, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(3, -1, -1, -1, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(4, 0, 1, 2, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(5, 0, 1, 1.5, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(6, 1, 0, 1, CoordinateType::KarthesianPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(7, 0, 1, 1, CoordinateType::KarthesianPointerToHull));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 1, 0, 0, CoordinateType::KarthesianPointerToHull));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(1, 0, 1, 0, CoordinateType::KarthesianPointerToHull));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(2, 0, 0, 1, CoordinateType::KarthesianPointerToHull));

			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
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
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}

		void siteThroughParametrizedAbsolute() {
			// setup
			SiteSpecArray ssa, ssa2, ssa3;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 5, M_PI / 2, 0, CoordinateType::ParametricAbsolute));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 5, M_PI / 2, M_PI / 2, CoordinateType::ParametricAbsolute));

			ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 3, CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(1, 2, 0, 0, CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(2, 2, M_PI/4, std::sqrt(2), CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(3, 2, M_PI * 5.0 / 4.0, -std::sqrt(2), CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(4, 1.5, M_PI / 2, 3, CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(5, 2, M_PI / 2, 3, CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(6, 2, 0, 2, CoordinateType::ParametricAbsolute));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(7, 2, M_PI / 2, 2, CoordinateType::ParametricAbsolute));

			ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricAbsolute));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(2, 1, 0, 0, CoordinateType::ParametricAbsolute));

			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
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
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}

		void siteThroughParamHullPointer() {
			// setup
			SiteSpecArray ssa, ssa2, ssa3;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));

			ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 3, CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(1, 2, 0, 0, CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(2, 2, M_PI / 4, std::sqrt(2), CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(3, 2, M_PI * 5.0 / 4.0, -std::sqrt(2), CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(4, 1.5, M_PI / 2, 3, CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(5, 2, M_PI / 2, 3, CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(6, 2, 0, 2, CoordinateType::ParametricPointerToHull));
			ssa2.push_back(CLibCollisionController::createSiteSpecification(7, 2, M_PI / 2, 2, CoordinateType::ParametricPointerToHull));

			ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));
			ssa3.push_back(CLibCollisionController::createSiteSpecification(2, 1, 0, 0, CoordinateType::ParametricPointerToHull));

			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
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
			BOOST_TEST(cc.displayAgent(0) == true);
			BOOST_TEST(cc.displayAgent(1) == true);
			BOOST_TEST(cc.displayAgent(2) == true);
		}

		void testAddingSites() {
			siteThroughKarthesianAbsolute();
			siteThroughKarthHullPointer();
			siteThroughParametrizedAbsolute();
			siteThroughParamHullPointer();
		}

		void testAgentTransformations() {
			//setup
			SiteSpecArray ssa;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI / 2, CoordinateType::ParametricPointerToHull));
			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);

			BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);

			// test cases
			try {
				std::shared_ptr<Agent> agnt = std::static_pointer_cast<simobj::Agent>(cc.getAgent(0));
				std::shared_ptr<Site> ste0 = std::static_pointer_cast<simobj::Site>(agnt->getSite(0));
				std::shared_ptr<Site> ste1 = std::static_pointer_cast<simobj::Site>(agnt->getSite(1));

				agnt->moveAgent(Vector3d(12, 0, 0));
				utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Global), "Agent translation test 1 - global frame");
				utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Local), "Agent translation test 2 - local frame");
				utils::compareCoordinates(Vector3d(17, 0, 0), ste0->getPosition(ReferenceFrame::Global), "Agent + site translation test 1 - global frame");
				utils::compareCoordinates(Vector3d(12, 5, 0), ste1->getPosition(ReferenceFrame::Global), "Agent + site translation test 2 - global frame");
				utils::compareCoordinates(Vector3d(5, 0, 0), ste0->getPosition(ReferenceFrame::Local), "Agent + site translation test 3 - local frame");
				utils::compareCoordinates(Vector3d(0, 5, 0), ste1->getPosition(ReferenceFrame::Local), "Agent + site translation test 4 - local frame");

				std::shared_ptr<Agent> agnt2 = std::static_pointer_cast<simobj::Agent>(cc.getAgent(2));
				std::shared_ptr<Site> ste02 = std::static_pointer_cast<simobj::Site>(agnt2->getSite(0));
				std::shared_ptr<Site> ste12 = std::static_pointer_cast<simobj::Site>(agnt2->getSite(1));

				agnt2->moveAgent(Vector3d(-12, 0, 0));
				agnt2->rotateAgent(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)));
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
				std::shared_ptr<Agent> agnt = std::static_pointer_cast<simobj::Agent>(cc.getAgent(0));
				std::shared_ptr<Site> ste0 = std::static_pointer_cast<simobj::Site>(agnt->getSite(0));
				std::shared_ptr<Site> ste1 = std::static_pointer_cast<simobj::Site>(agnt->getSite(1));

				utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Global), "Agent translation test 3 - global frame");
				utils::compareCoordinates(Vector3d(12, 0, 0), agnt->getPosition(ReferenceFrame::Local), "Agent translation test 4 - local frame");
				utils::compareCoordinates(Vector3d(17, 0, 0), ste0->getPosition(ReferenceFrame::Global), "Agent + site translation test 3 - global frame");
				utils::compareCoordinates(Vector3d(12, 5, 0), ste1->getPosition(ReferenceFrame::Global), "Agent + site translation test 4 - global frame");
				utils::compareCoordinates(Vector3d(5, 0, 0), ste0->getPosition(ReferenceFrame::Local), "Agent + site translation test 5 - local frame");
				utils::compareCoordinates(Vector3d(0, 5, 0), ste1->getPosition(ReferenceFrame::Local), "Agent + site translation test 6 - local frame");

				std::shared_ptr<Agent> agnt2 = std::static_pointer_cast<simobj::Agent>(cc.getAgent(2));
				std::shared_ptr<Site> ste02 = std::static_pointer_cast<simobj::Site>(agnt2->getSite(0));
				std::shared_ptr<Site> ste12 = std::static_pointer_cast<simobj::Site>(agnt2->getSite(1));

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

			BOOST_TEST(cc.displayAgentCluster(0) == true);
		}

		void testAgentClusterTransformations() {
			//setup
			SiteSpecArray ssa;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);

			BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(3, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(4, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(5, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(6, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(7, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(8, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(9, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(10, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(11, "SphereAgent") == true);

			// test cases
			try {
				cc.connectAgents(0, 1, 0, 1);
				cc.connectAgents(1, 2, 0, 1);
				cc.connectAgents(2, 3, 0, 1);
				cc.connectAgents(3, 4, 0, 1);
				cc.connectAgents(4, 5, 0, 1);
				cc.connectAgents(5, 6, 0, 1);
				cc.connectAgents(6, 7, 0, 1);
				cc.connectAgents(7, 8, 0, 1);
				cc.connectAgents(8, 9, 0, 1);
				cc.connectAgents(9, 10, 0, 1);
				cc.connectAgents(10, 11, 0, 1);

			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
			}

			BOOST_TEST(cc.displayAgentCluster(0) == true);

		}

		void testTransformations() {
			testAgentTransformations();
			testAgentClusterTransformations();
		}

	}
}

namespace collision {
	namespace octtree {
		namespace utils {
			static const double EPS = 0.00001;
		}

		using Eigen::Vector3d;
		using Quaternion = Eigen::Quaternion<double>;
		using simobj::shapes::ShapePtr;
		using simobj::specs::MetaSpecification;
		using simobj::specs::AgentSpecification;
		using simobj::specs::SiteSpecification;
		using SiteSpecArray = std::vector<SiteSpecification>;
		using AgentSpecArray = std::vector<AgentSpecification>;
		using ShapesArray = std::vector<ShapePtr>;
		using clib::CLibCollisionController;
		using simobj::specs::CoordinateType;
		using simobj::shapes::ShapeType;

		void testOctTreeConstrution(){
			/*
			TreePtr<double> tree = OctTree<double>::create(Bounds(-5, -5 , -5), Bounds(5, 5, 5), Bounds(1.25, 1.25, 1.25));
			tree->insertNode(0, Bounds(1, 0, 0), Bounds(2, 1, 1));
			tree->insertNode(1, Bounds(1, 0.5, 0.5), Bounds(1.5, 1, 1));
			tree->insertNode(2, Bounds(-0.5, -0.5, -0.5), Bounds(0.5, 0.5, 0.5));
			std::cout << "Nearest Distance: " << tree->getNearestDistance(1, -2, 0) << std::endl;
			*/
			//setup
			SiteSpecArray ssa;
			ssa.push_back(CLibCollisionController::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
			ssa.push_back(CLibCollisionController::createSiteSpecification(1, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
			ShapesArray sa;
			sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 1.0));
			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);

			BOOST_TEST(cc.createAgent(0, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(1, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(2, "SphereAgent") == true);
			BOOST_TEST(cc.createAgent(3, "SphereAgent") == true);

			// test cases
			try {
				cc.connectAgents(0, 1, 0, 1);
				cc.connectAgents(1, 2, 0, 1);
				cc.connectAgents(2, 3, 0, 1);

			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
			}
			BOOST_TEST(cc.addAgentClusterToCollisionDetector(0) == true);
			BOOST_TEST(cc.displayClusterCollisionTree(0) == true);
		}
	}
}

BOOST_AUTO_TEST_SUITE(Simulation_Objects_test_suite)
/*
BOOST_AUTO_TEST_CASE(Test_Exceptions, *boost::unit_test::tolerance(simobj::tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(simobj::tests::testExceptions());
}

BOOST_AUTO_TEST_CASE(Test_Adding_Sites, * boost::unit_test::tolerance(simobj::tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(simobj::tests::testAddingSites());
}

BOOST_AUTO_TEST_CASE(Test_Transformations, *boost::unit_test::tolerance(simobj::tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(simobj::tests::testTransformations());
}
*/
BOOST_AUTO_TEST_CASE(Test_OctTeeLibrary, *boost::unit_test::tolerance(collision::octtree::utils::EPS))
{
	BOOST_CHECK_NO_THROW(collision::octtree::testOctTreeConstrution());
}

BOOST_AUTO_TEST_SUITE_END()

