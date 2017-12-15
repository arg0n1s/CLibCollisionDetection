// TestApplication.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <tchar.h>
#include <SimulationContainer.h>
#include <MetaSpecification.h>
#include <Shape.h>
#include <iostream>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <CLibCollisionController.h>

using Eigen::Vector3d;
using simobj::shapes::ShapePtr;
using simobj::specs::MetaSpecification;
using simobj::specs::AgentSpecification;
using simobj::specs::SiteSpecification;
using SiteSpecArray = std::vector<SiteSpecification>;
using AgentSpecArray = std::vector<AgentSpecification>;
using ShapesArray = std::vector<ShapePtr>;
using clib::CLibCollisionController;

int main()
{
	/*
	//create meta specifications
	specs::MetaSpecification meta;
	shapes::ShapePtr shape = shapes::ShapeFactory::create<double>(shapes::ShapeType::Sphere, 5.0);
	specs::AgentSpecification agentSpec("dings", shape);
	agentSpec.addSiteSpecification(specs::SiteSpecification(0, "bums", Vector3d(0, 0, 0)));
	meta.addAgentSpecification(agentSpec);
	//create simulation container
	SimulationContainer container(meta);
	container.addAgent(0, "dings");
	container.addAgent(1, "dings");
	container.addAgent(2, "dings");
	container.addAgent(3, "dings");
	//print contents
	std::cout << container.toString();
	*/
	SiteSpecArray ssa, ssa2, ssa3;
	ssa.push_back(CLibCollisionController::createSiteSpecification(0, "bums", 5, 0, 0));
	ssa.push_back(CLibCollisionController::createSiteSpecification(1, "bums", 0, 5, 0));
	ssa2.push_back(CLibCollisionController::createSiteSpecification(0, "bums2", 0, 0, 3));
	ssa2.push_back(CLibCollisionController::createSiteSpecification(1, "bums2", 2, 0, 0));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(0, "bums3", 2, 0, 0));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(1, "bums3", 0, 3, 0));
	ssa3.push_back(CLibCollisionController::createSiteSpecification(2, "bums3", 0, 0, 1));
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
	cc.createAgent(1, "dings2");
	cc.createAgent(2, "dings3");
	std::cout << cc.toString();
	cc.displayAgent(0);
	cc.displayAgent(1);
	cc.displayAgent(2);
    return 0;
}

