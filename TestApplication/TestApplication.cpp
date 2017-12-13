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

using Eigen::Vector3d;
using namespace simobj;

int main()
{
	//create meta specifications
	specs::MetaSpecification meta;
	std::shared_ptr<shapes::Shape> shape = std::shared_ptr<shapes::Shape>(new shapes::Sphere(5.0));
	specs::AgentSpecification agentSpec("dings", shape);
	agentSpec.addSiteSpecification(specs::SiteSpecification(0, "bums", Vector3d(0, 0, 0)));
	meta.addAgentSpecification(agentSpec);
	//create simulation container
	SimulationContainer container(meta);
	container.addAgent(0, "dings");
	container.addAgent(1, "dings");
	container.addAgent(2, "dings");
	container.addAgent(3, "dings");
	
    return 0;
}

