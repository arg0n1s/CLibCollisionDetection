#include "../include/CLibCollisionController.h"

#include <memory>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <Shape.h>
#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>

#include <iostream>

namespace clib {

	using Eigen::Vector3d;
	using Quaternion = Eigen::Quaternion<double>;
	using simobj::shapes::ShapeFactory;
	using simobj::shapes::ShapePtr;
	using simobj::shapes::ShapeType;
	using std::shared_ptr;
	using simobj::Agent;
	using simobj::Site;
	using simobj::AgentCluster;

	CLIB_COLLISION_DETECTION_API SiteSpecification CLibCollisionController::createSiteSpecification(const unsigned long& id, const string& type, const double& x, const double& y, const double& z) {
		return SiteSpecification(id, type, Vector3d(x,y,z));
	}

	template<typename... Args>
	CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const unsigned int shapeType, Args... args) {
		return ShapeFactory::create(static_cast<ShapeType>(shapeType), args...);
	}

	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const unsigned int, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const unsigned int, double, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const unsigned int, double, double, double);

	CLIB_COLLISION_DETECTION_API AgentSpecification CLibCollisionController::createAgentSpecification(const string& type, ShapePtr shape, SiteSpecArray siteSpecs) {
		AgentSpecification agent(type, shape);
		for (auto sitespec : siteSpecs) {
			agent.addSiteSpecification(sitespec);
		}
		return agent;
	}

	CLIB_COLLISION_DETECTION_API MetaSpecification CLibCollisionController::createMetaSpecification(AgentSpecArray agentSpecs) {
		MetaSpecification meta;
		for (auto agentspec : agentSpecs) {
			meta.addAgentSpecification(agentspec);
		}
		return meta;
	}

	CLIB_COLLISION_DETECTION_API CLibCollisionController::CLibCollisionController(const MetaSpecification& metaSpecs) : metaSpecs(metaSpecs) {
		simContainer = SimulationContainer(metaSpecs);
		vtkVis = VTKVisualization();
		clusterCounter = 0;
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::createAgent(const unsigned long& id, const string& type) {
		simContainer.addAgent(id, type);
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::connectAgents(const unsigned long& agt1, const unsigned long& agt2, const unsigned long& st1, const unsigned long& st2) {
		shared_ptr<Agent> agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt1));
		shared_ptr<Agent> agent2 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt2));
		shared_ptr<Site> site1 = std::static_pointer_cast<Site>(agent1->getSite(st1));
		shared_ptr<Site> site2 = std::static_pointer_cast<Site>(agent2->getSite(st2));

		Vector3d agentOrigin1, agentOrigin2, siteOrigin1, siteOrigin2, s1ToO1, s2ToO2, O1ToS1, O2toS2;
		agentOrigin1 = agent1->getPosition();
		agentOrigin2 = agent2->getPosition();
		siteOrigin1 = agent1->getConvertedPosition(site1->getPosition());
		siteOrigin2 = agent2->getConvertedPosition(site2->getPosition());

		std::cout << "Site1: " << siteOrigin1 << std::endl;
		std::cout << "Site2: " << siteOrigin2 << std::endl;

		s1ToO1 = (agentOrigin1 - siteOrigin1).normalized();
		O2toS2 = (siteOrigin2 - agentOrigin2).normalized();

		std::cout << "Site1 to Origin1: " << s1ToO1 << std::endl;
		std::cout << "Origin2 to Site2: " << O2toS2 << std::endl;

		Quaternion rot = Quaternion::FromTwoVectors(O2toS2, s1ToO1);
		agent2->rotateAgent(rot);
		siteOrigin2 = agent2->getConvertedPosition(site2->getPosition());
		std::cout << "Site2: " << siteOrigin2 << std::endl;

		s2ToO2 = agentOrigin2 - siteOrigin2;
		O1ToS1 = siteOrigin1 - agentOrigin1;
		agent2->setPosition(agentOrigin1 + O1ToS1 + s2ToO2);

		std::cout << "Agent2: " << agent2->getPosition() << std::endl;

		simContainer.addAgentCluster(clusterCounter, "default");
		
		shared_ptr<AgentCluster> cluster = std::static_pointer_cast<AgentCluster>(simContainer.getAgentCluster(clusterCounter));
		cluster->insertAgent(agent1);
		cluster->insertAgent(agent2);
		site1->connect(site2);

		clusterCounter++;
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::displayAgent(const unsigned long& id) {
		vtkVis.renderAgent(simContainer.getAgent(id));
		vtkVis.display();
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::displayAgentCluster(const unsigned long& id) {
		vtkVis.renderAgentCluster(simContainer.getAgentCluster(id));
		vtkVis.display();
	}

	CLIB_COLLISION_DETECTION_API string CLibCollisionController::toString() {
		return simContainer.toString();
	}
}