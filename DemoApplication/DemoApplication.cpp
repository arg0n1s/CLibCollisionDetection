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

#define _USE_MATH_DEFINES
#include <math.h>

using Eigen::Vector3d;
using Quaternion = Eigen::Quaternion<double>;

using namespace clib;
using ShapesArray = std::vector<ShapePtr>;
using simobj::Agent;
using simobj::AgentCluster;

void demo() {
	//setup
	SiteSpecArray ssa;
	ssa.push_back(clib::createSiteSpecification(0, 1, M_PI / 2, 0, CoordinateType::ParametricPointerToHull));
	ssa.push_back(clib::createSiteSpecification(1, 1, 0, 0, CoordinateType::ParametricPointerToHull));
	ssa.push_back(clib::createSiteSpecification(2, 1, M_PI / 2, M_PI * 3.0 / 4.0, CoordinateType::ParametricPointerToHull));
	SiteSpecArray ssa2;
	ssa2.push_back(clib::createSiteSpecification(0, 0, 0, 1, CoordinateType::ParametricPointerToHull));
	ssa2.push_back(clib::createSiteSpecification(1, 0, 0, -1, CoordinateType::ParametricPointerToHull));
	ShapesArray sa;
	sa.push_back(CLibCollisionController::createShape(ShapeType::Sphere, 3.0));
	sa.push_back(CLibCollisionController::createShape(ShapeType::Cylinder, 0.5, 12.0));
	AgentSpecArray asa;
	asa.push_back(clib::createAgentSpecification("SphereAgent", sa[0], ssa));
	asa.push_back(clib::createAgentSpecification("CylinderAgent", sa[1], ssa2));
	MetaSpecification meta = clib::createMetaSpecification(asa);
	CLibCollisionController cc(meta);
	cc.setInitialRootDiameter(18.0);
	cc.setMinimalLeafDiameter(9.0);

	cc.createAgent(0, "SphereAgent");
	cc.createAgent(1, "SphereAgent");
	cc.createAgent(2, "SphereAgent");
	cc.createAgent(3, "SphereAgent");
	cc.createAgent(4, "SphereAgent"); 
	cc.createAgent(5, "SphereAgent");
	cc.createAgent(6, "SphereAgent");
	cc.createAgent(7, "SphereAgent");
	cc.createAgent(8, "SphereAgent");
	cc.createAgent(9, "SphereAgent"); 
	cc.createAgent(10, "SphereAgent");
	cc.createAgent(11, "SphereAgent"); 
	cc.createAgent(12, "SphereAgent");
	cc.createAgent(13, "SphereAgent");
	cc.createAgent(14, "SphereAgent");
	cc.createAgent(15, "SphereAgent");
	cc.createAgent(16, "CylinderAgent");
	cc.createAgent(17, "CylinderAgent");
	cc.createAgent(18, "CylinderAgent");
	cc.createAgent(19, "CylinderAgent");
	cc.createAgent(20, "CylinderAgent");
	cc.createAgent(21, "CylinderAgent");
	cc.createAgent(22, "CylinderAgent");
	cc.createAgent(23, "CylinderAgent");

	//stage 1: Scene setup -> 2 agents
	cc.createAgent(24, "SphereAgent");
	cc.createAgent(25, "SphereAgent");
	std::shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(cc.getAgent(25));
	agnt->setPosition(Vector3d(-9, 6, 6));
	cc.createAgentCluster(10, "show");
	std::shared_ptr<AgentCluster> clst = std::static_pointer_cast<AgentCluster>(cc.getAgentCluster(10));
	clst->insertAgent(agnt);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(24));
	clst->insertAgent(agnt);
	cc.displayAgentCluster(10);
	//stage 2: show connect
	cc.createAgent(26, "SphereAgent");
	cc.createAgent(27, "SphereAgent");
	cc.connectAgents(26, 27, 0, 2);
	cc.displayAgentCluster(0);
	//stage 3: show new candidate (cylinder)
	cc.createAgent(28, "CylinderAgent");
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(28));
	agnt->setPosition(Vector3d(-9, 6, 6));
	clst = std::static_pointer_cast<AgentCluster>(cc.getAgentCluster(0));
	clst->insertAgent(agnt);
	cc.displayAgentCluster(0);
	//stage 4: connect and show octTree
	cc.createAgent(29, "SphereAgent");
	cc.createAgent(30, "SphereAgent");
	cc.createAgent(31, "CylinderAgent");
	cc.connectAgents(29, 30, 0, 2);
	cc.addAgentClusterToCollisionDetector(1);
	cc.connectAgents(29, 31, 1, 0);
	cc.displayClusterCollisionTree(1);
	//from here on continue like usual
	try {
	cc.connectAgents(0, 1, 0, 2);
	cc.connectAgents(0, 16, 1, 0);
	cc.connectAgents(1, 2, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(2, 3, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(3, 4, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(4, 5, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(5, 6, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(6, 7, 0, 2);
	cc.displayAgentCluster(2);
	cc.connectAgents(1, 17, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(2, 18, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(3, 19, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(4, 20, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(5, 21, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(6, 22, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(7, 23, 1, 0);
	cc.displayAgentCluster(2);
	cc.connectAgents(16, 8, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(8));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(-1, -1, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(17, 9, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(9));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(-1, 0, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(18, 10, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(10));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(-1, 1, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(19, 11, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(11));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, 1, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(20, 12, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(12));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(1, 1, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(21, 13, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(13));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(1, 0, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(22, 14, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(14));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(1, -1, 0)));
	cc.displayAgentCluster(2);
	cc.connectAgents(23, 15, 1, 1);
	agnt = std::static_pointer_cast<Agent>(cc.getAgent(15));
	agnt->rotate(Quaternion::FromTwoVectors(Vector3d(1, 0, 0), Vector3d(0, -1, 0)));
	cc.displayAgentCluster(2);

	}
	catch (std::exception& e) {
	std::cout << e.what() << std::endl;
	}
	cc.addAgentClusterToCollisionDetector(2);
	cc.displayClusterCollisionTree(2);

}


int main()
{
	demo();
    return 0;
}

