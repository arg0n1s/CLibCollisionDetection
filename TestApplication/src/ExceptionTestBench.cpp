#include "..\include\ExceptionTestBench.h"
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
	using simobj::shapes::ShapePtr;
	using simobj::Agent;
	using namespace clib;

	vector<SiteSpecification> ptrArrayToValueArray(const vector<SiteSpecPtr>& siteSpecs) {
		vector<SiteSpecification> specs;
		for (int i = 0; i < siteSpecs.size(); i++) {
			specs.push_back(*siteSpecs.at(i));
		}
		return specs;
	}

	vector<AgentSpecification> ptrArrayToValueArray(const vector<AgentSpecPtr>& agentSpecs) {
		vector<AgentSpecification> specs;
		for (int i = 0; i < agentSpecs.size(); i++) {
			specs.push_back(*agentSpecs.at(i));
		}
		return specs;
	}

	void ExceptionTestBench::setup() {
		// setup1
		vector<SiteSpecPtr> ssa, ssa2, ssa3, ssa4, ssa5, ssa6;
		ssa.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute)));
		ssa2.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 0, 0, 3, CoordinateType::KarthesianAbsolute)));
		ssa3.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 2, 0, 0, CoordinateType::KarthesianAbsolute)));
		ssa4.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 5, 0, 0, CoordinateType::KarthesianAbsolute)));
		ssa4.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 5, M_PI / 2, 0, CoordinateType::KarthesianAbsolute)));
		ssa5.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 5, M_PI / 2, 0, static_cast<CoordinateType>(4))));
		ssa6.push_back(std::make_unique<SiteSpecification>(clib::createSiteSpecification(0, 0, 0, 0, CoordinateType::KarthesianPointerToHull)));
		siteSpecs.insert(std::make_pair("SiteSpecs1", ssa));
		siteSpecs.insert(std::make_pair("SiteSpecs2", ssa2));
		siteSpecs.insert(std::make_pair("SiteSpecs3", ssa3));
		siteSpecs.insert(std::make_pair("SiteSpecs4", ssa4));
		siteSpecs.insert(std::make_pair("SiteSpecs5", ssa5));
		siteSpecs.insert(std::make_pair("SiteSpecs6", ssa6));

		// setup2
		vector<ShapePtr> shapeArray;
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 5.0));
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 2.0, 6.0));
		shapeArray.push_back(CLibCollisionController::createShape(ShapeType::Ellipsoid, 2.0, 3.0, 1.0));
		shapes.insert(std::make_pair("shapes1", shapeArray));

		vector<AgentSpecPtr> agentSpecArray, agentSpecArray2, agentSpecArray3;
		agentSpecArray.push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ptrArrayToValueArray(siteSpecs["SiteSpecs1"]))));
		agentSpecArray.push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("CylinderAgent", shapes["shapes1"][1], ptrArrayToValueArray(siteSpecs["SiteSpecs2"]))));
		agentSpecArray.push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("EllipsoidAgent", shapes["shapes1"][2], ptrArrayToValueArray(siteSpecs["SiteSpecs3"]))));
		agentSpecs.insert(std::make_pair("AgentSpecs1", agentSpecArray));

		// setup3
		metaSpecs.insert(std::make_pair("MetaSpecs1", std::make_unique<MetaSpecification>(clib::createMetaSpecification(ptrArrayToValueArray(agentSpecs["AgentSpecs1"])))));

		// setup4
		CLibController.insert(std::make_pair("CLib1", std::make_unique<CLibCollisionController>(*metaSpecs["MetaSpecs1"])));
		CLibController["CLib1"]->createAgent(0, "SphereAgent");
		CLibController["CLib1"]->createAgent(1, "CylinderAgent");
		CLibController["CLib1"]->createAgent(2, "EllipsoidAgent");

		agentSpecArray2.push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("SphereAgent1", shapes["shapes1"][0], ptrArrayToValueArray(siteSpecs["SiteSpecs5"]))));
		agentSpecs.insert(std::make_pair("AgentSpecs2", agentSpecArray2));
		metaSpecs.insert(std::make_pair("MetaSpecs2", std::make_unique<MetaSpecification>(clib::createMetaSpecification(ptrArrayToValueArray(agentSpecs["AgentSpecs2"])))));
		CLibController.insert(std::make_pair("CLib2", std::make_unique<CLibCollisionController>(*metaSpecs["MetaSpecs2"])));

		agentSpecArray3.push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("SphereAgent2", shapes["shapes1"][0], ptrArrayToValueArray(siteSpecs["SiteSpecs6"]))));
		agentSpecs.insert(std::make_pair("AgentSpecs3", agentSpecArray3));
		metaSpecs.insert(std::make_pair("MetaSpecs3", std::make_unique<MetaSpecification>(clib::createMetaSpecification(ptrArrayToValueArray(agentSpecs["AgentSpecs3"])))));
		CLibController.insert(std::make_pair("CLib3", std::make_unique<CLibCollisionController>(*metaSpecs["MetaSpecs3"])));
	}

	void ExceptionTestBench::runAllTests() {
		std::cout << "*********** Exception tests begin, i.e. all following Error messages are intentional: \n" << std::endl;

		bool shapeTest1 = false, shapeTest2 = false, shapeTest3 = false, shapeTest4 = false, shapeTest5 = false, shapeTest6 = false;

		// trigger shape creation exceptions
		try {
			CLibCollisionController::createShape(ShapeType::Sphere, 0.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest1 = true;
		}
		try {
			CLibCollisionController::createShape(ShapeType::Cylinder, 0.0, 1.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest2 = true;
		}
		try {
			CLibCollisionController::createShape(ShapeType::Ellipsoid, 0.0, 1.0, 1.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest3 = true;
		}
		try {
			CLibCollisionController::createShape(ShapeType::Sphere, 0.0, 0.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest4 = true;
		}
		try {
			CLibCollisionController::createShape(ShapeType::Cylinder, 0.0, 1.0, 1.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest5 = true;
		}
		try {
			CLibCollisionController::createShape(ShapeType::Ellipsoid, 0.0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			shapeTest6 = true;
		}

		//trigger agent spec exceptions
		bool agentSpecTest1 = false;
		
		try {
			clib::createAgentSpecification("SphereAgent1", shapes["shapes1"][0], ptrArrayToValueArray(siteSpecs["SiteSpecs4"]));
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			agentSpecTest1 = true;
		}

		//trigger meta spec exceptions
		bool metaSpecTest1 = false;
		//trigger exceptions
		try {
			agentSpecs["AgentSpecs1"].push_back(std::make_unique<AgentSpecification>(clib::createAgentSpecification("SphereAgent", shapes["shapes1"][0], ptrArrayToValueArray(siteSpecs["SiteSpecs1"]))));
			MetaSpecification meta2 = clib::createMetaSpecification(ptrArrayToValueArray(agentSpecs["AgentSpecs1"]));
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

		//trigger simulation container exceptions
		bool simulationContainerTest1 = false, simulationContainerTest2 = false, simulationContainerTest3 = false, simulationContainerTest4 = false, simulationContainerTest5 = false;
		
		BOOST_TEST(CLibController["CLib1"]->createAgent(0, "SphereAgent") == false, "Simulation container test 1 did not trigger the expected exception!");
		BOOST_TEST(CLibController["CLib1"]->createAgent(3, "SphereAgent0") == false, "Simulation container test 2 did not trigger the expected exception!");
		BOOST_TEST(CLibController["CLib1"]->createAgent(4, "SphereAgent1") == false, "Simulation container test 3 did not trigger the expected exception!");
		BOOST_TEST(CLibController["CLib1"]->createAgent(5, "SphereAgent2") == false, "Simulation container test 4 did not trigger the expected exception!");
		try {
			CLibController["CLib1"]->getAgent(4);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			simulationContainerTest1 = true;
		}
		try {
			std::static_pointer_cast<simobj::Agent>(CLibController["CLib1"]->getAgent(0))->getSite(5);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			simulationContainerTest2 = true;
		}
		BOOST_TEST(simulationContainerTest1 == true, "Simulation container test 5 did not trigger the expected exception!");
		BOOST_TEST(simulationContainerTest2 == true, "Simulation container test 6 did not trigger the expected exception!");
		BOOST_TEST(CLibController["CLib1"]->displayAgent(6) == false, "Simulation container test 7 did not trigger the expected exception!");
		try {
			CLibController["CLib1"]->connectAgents(3, 2, 0, 0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			simulationContainerTest3 = true;
		}
		try {
			CLibController["CLib1"]->connectAgents(1, 2, 5, 0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			simulationContainerTest4 = true;
		}
		try {
			CLibController["CLib1"]->connectAgents(1, 1, 0, 0);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			simulationContainerTest5 = true;
		}
		BOOST_TEST(simulationContainerTest3 == true, "Simulation container test 8 did not trigger the expected exception!");
		BOOST_TEST(simulationContainerTest4 == true, "Simulation container test 9 did not trigger the expected exception!");
		BOOST_TEST(simulationContainerTest5 == true, "Simulation container test 10 did not trigger the expected exception!");
		std::cout << "\n\n*********** Exception tests end." << std::endl;
	}

}