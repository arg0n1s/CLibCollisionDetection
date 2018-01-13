#pragma once
#define BOOST_TEST_MODULE TestApplication

#include <boost/test/included/unit_test.hpp>
#include <SimulationContainer.h>
#include <Agent.h>
#include <MetaSpecification.h>
#include <Shape.h>
#include <CLibCollisionController.h>
/*
#include <stdio.h>
#include <tchar.h>
#include <SimulationContainer.h>
#include <MetaSpecification.h>
#include <Shape.h>
#include <iostream>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <CLibCollisionController.h>
*/

/*
using Eigen::Vector3d;
using simobj::shapes::ShapePtr;
using simobj::specs::MetaSpecification;
using simobj::specs::AgentSpecification;
using simobj::specs::SiteSpecification;
using SiteSpecArray = std::vector<SiteSpecification>;
using AgentSpecArray = std::vector<AgentSpecification>;
using ShapesArray = std::vector<ShapePtr>;
using clib::CLibCollisionController;
using simobj::specs::CoordinateType;
*/
/*
BOOST_AUTO_TEST_CASE(test_case)
{
	
	SiteSpecArray ssa, ssa2, ssa3;
	ssa.push_back(CLibCollisionController::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute));
	ssa.push_back(CLibCollisionController::createSiteSpecification(1, 0, 5, 0, CoordinateType::KarthesianAbsolute));
	ssa2.push_back(CLibCollisionController::createSiteSpecification(0, 0, 0, 3, CoordinateType::KarthesianAbsolute));
	ssa2.push_back(CLibCollisionController::createSiteSpecification(1, 2, 0, 0, CoordinateType::KarthesianAbsolute));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(0, 2, 0, 0, CoordinateType::KarthesianAbsolute));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(1, 0, 3, 0, CoordinateType::KarthesianAbsolute));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(2, 0, 0, 1, CoordinateType::KarthesianAbsolute));
	ShapesArray sa;
	sa.push_back(CLibCollisionController::createShape(0, 5.0));
	sa.push_back(CLibCollisionController::createShape(1, 2.0, 6.0));
	sa.push_back(CLibCollisionController::createShape(2, 2.0, 3.0, 1.0));
	AgentSpecArray asa;
	asa.push_back(CLibCollisionController::createAgentSpecification("dings", sa[0], ssa));
	asa.push_back(CLibCollisionController::createAgentSpecification("dings2", sa[1], ssa2));
	asa.push_back(CLibCollisionController::createAgentSpecification("dings3", sa[2], ssa3));
	MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
	CLibCollisionController cc(meta);
	cc.createAgent(0, "dings");
	cc.createAgent(1, "dings");
	cc.createAgent(2, "dings");
	cc.createAgent(3, "dings");
	cc.createAgent(4, "dings2");
	cc.createAgent(5, "dings3");
	
	//cc.displayAgent(0);
	//cc.displayAgent(1);
	//cc.displayAgent(2);
	cc.connectAgents(1, 2, 0, 0);
	cc.connectAgents(0, 3, 0, 0);
	cc.connectAgents(4, 5, 0, 1);
	//std::cout << cc.toString();
	cc.displayAgentCluster(0);
	cc.displayAgentCluster(1);
	cc.displayAgentCluster(2);

	return 0;
	
	BOOST_CHECK_NO_THROW(simobj::tests::testAddingSites());

}
*/

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

		namespace utils {
			static const double EPS = 0.00001;

			void compareCoordinates(const Vector3d& correct, const Vector3d& given, const std::string& testname) {
				//BOOST_TEST((lhs - rhs).norm() <= EPS);
				//std::cout << "\n Left Coordinates: \n" << lhs << std::endl;
				//std::cout << "\n Right Corrdinates: \n" << rhs << std::endl;
				BOOST_TEST(correct.x() == given.x(), "\n Error for x-coordinates in: " << testname << ". Given x value was: " << given.x() << ", but should have been: " << correct.x() << ".");
				BOOST_TEST(correct.y() == given.y(), "\n Error for y-coordinates in: " << testname << ". Given y value was: " << given.y() << ", but should have been: " << correct.y() << ".");
				BOOST_TEST(correct.z() == given.z(), "\n Error for z-coordinates in: " << testname << ". Given z value was: " << given.z() << ", but should have been: " << correct.z() << ".");
			}

			void compareOrientation(const Quaternion& lhs, const Quaternion& rhs) {
				static const Vector3d xAxis(1, 0, 0);
				BOOST_TEST((lhs*xAxis - rhs*xAxis).norm() <= EPS);
			}
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
			sa.push_back(CLibCollisionController::createShape(0, 5.0));
			sa.push_back(CLibCollisionController::createShape(1, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(2, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);

			BOOST_TEST(cc.createAgent(0, "SphereAgent"));
			BOOST_TEST(cc.createAgent(1, "CylinderAgent"));
			BOOST_TEST(cc.createAgent(2, "EllipsoidAgent"));

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
			BOOST_TEST(cc.displayAgent(0));
			BOOST_TEST(cc.displayAgent(1));
			BOOST_TEST(cc.displayAgent(2));
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
			sa.push_back(CLibCollisionController::createShape(0, 5.0));
			sa.push_back(CLibCollisionController::createShape(1, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(2, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
			BOOST_TEST(cc.createAgent(0, "SphereAgent"));
			BOOST_TEST(cc.createAgent(1, "CylinderAgent"));
			BOOST_TEST(cc.createAgent(2, "EllipsoidAgent"));

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
			BOOST_TEST(cc.displayAgent(0));
			BOOST_TEST(cc.displayAgent(1));
			BOOST_TEST(cc.displayAgent(2));
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
			sa.push_back(CLibCollisionController::createShape(0, 5.0));
			sa.push_back(CLibCollisionController::createShape(1, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(2, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
			BOOST_TEST(cc.createAgent(0, "SphereAgent"));
			BOOST_TEST(cc.createAgent(1, "CylinderAgent"));
			BOOST_TEST(cc.createAgent(2, "EllipsoidAgent"));

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
			BOOST_TEST(cc.displayAgent(0));
			BOOST_TEST(cc.displayAgent(1));
			BOOST_TEST(cc.displayAgent(2));
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
			sa.push_back(CLibCollisionController::createShape(0, 5.0));
			sa.push_back(CLibCollisionController::createShape(1, 2.0, 6.0));
			sa.push_back(CLibCollisionController::createShape(2, 2.0, 3.0, 1.0));

			AgentSpecArray asa;
			asa.push_back(CLibCollisionController::createAgentSpecification("SphereAgent", sa[0], ssa));
			asa.push_back(CLibCollisionController::createAgentSpecification("CylinderAgent", sa[1], ssa2));
			asa.push_back(CLibCollisionController::createAgentSpecification("EllipsoidAgent", sa[2], ssa3));

			MetaSpecification meta = CLibCollisionController::createMetaSpecification(asa);
			CLibCollisionController cc(meta);
			BOOST_TEST(cc.createAgent(0, "SphereAgent"));
			BOOST_TEST(cc.createAgent(1, "CylinderAgent"));
			BOOST_TEST(cc.createAgent(2, "EllipsoidAgent"));

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
			BOOST_TEST(cc.displayAgent(0));
			BOOST_TEST(cc.displayAgent(1));
			BOOST_TEST(cc.displayAgent(2));
		}

		void testAddingSites() {
			siteThroughKarthesianAbsolute();
			siteThroughKarthHullPointer();
			siteThroughParametrizedAbsolute();
			siteThroughParamHullPointer();
		}

		void testRotation() {

		}

		void testTranslation() {

		}

		void testTransformations() {
			//BOOST_CHECK_NO_THROW(testRotation());
			//BOOST_CHECK_NO_THROW(testTranslation());
		}

	}
}

BOOST_AUTO_TEST_SUITE(Simulation_Objects_test_suite)

BOOST_AUTO_TEST_CASE(Test_Adding_Sites, * boost::unit_test::tolerance(simobj::tests::utils::EPS))
{
	BOOST_CHECK_NO_THROW(simobj::tests::testAddingSites());
}


BOOST_AUTO_TEST_SUITE_END()

