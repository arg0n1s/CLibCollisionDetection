#include "../include/CLibCollisionController.h"
#include "../include/CLibErrorLogger.h"

#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <OctTree.h>

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
	using simobj::ReferenceFrame;

	CLIB_COLLISION_DETECTION_API SiteSpecification CLibCollisionController::createSiteSpecification(const unsigned long& id, const double& c1, const double& c2, const double& c3, const CoordinateType& cType) {
		return SiteSpecification(id, "default", Vector3d(c1,c2,c3), cType);
	}

	template<typename... Args>
	CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType& shapeType, Args... args) {
		return ShapeFactory::create(shapeType, args...);
	}

	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double, double, double);

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
		ErrorLogger::instance();
		simContainer = SimulationContainer(metaSpecs);
		collisionDetector = CollisionDetection();
		vtkVis = VTKVisualization();
		clusterCounter = 0;
	}

	CLIB_COLLISION_DETECTION_API CLibCollisionController::~CLibCollisionController() {
		
		try {
			ErrorLogger::instance().saveToLogToFile();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
		}

	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::createAgentCluster(const unsigned long& id, const string& type) {
		try {
			simContainer.addAgentCluster(id, type);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::createAgent(const unsigned long& id, const string& type) {
		try {
			simContainer.addAgent(id, type);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API SimObjPtr CLibCollisionController::getAgent(const unsigned long& id) {
		return simContainer.getAgent(id);
	}
	CLIB_COLLISION_DETECTION_API SimObjPtr CLibCollisionController::getAgentCluster(const unsigned long& id) {
		return simContainer.getAgentCluster(id);
	}

	CLIB_COLLISION_DETECTION_API CollisionDetection& CLibCollisionController::getCollisionDetector() {
		return collisionDetector;
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId) {
		try {
			simContainer.addAgentToCluster(agentId, clusterId);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::addAgentClusterToCollisionDetector(const unsigned long& clusterId) {
		try {
			SimObjPtr clstr = getAgentCluster(clusterId);
			collisionDetector.setAllowRescaling(true);
			collisionDetector.setMinimalCellDiameter(2.0);
			collisionDetector.setInitialTreeDiameter(4.0);
			collisionDetector.makeTreeFromCluster(clstr);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::connectAgents(const unsigned long& agt1, const unsigned long& agt2, const unsigned long& st1, const unsigned long& st2) {
		shared_ptr<Agent> agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt1));
		shared_ptr<Agent> agent2 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt2));
		if (agent1->getId() == agent2->getId()) throw std::runtime_error("Error, cannot connect identical agents!");
		
		shared_ptr<Site> site1 = std::static_pointer_cast<Site>(agent1->getSite(st1));
		shared_ptr<Site> site2 = std::static_pointer_cast<Site>(agent2->getSite(st2));

		Vector3d agentOrigin1, agentOrigin2, siteOrigin1, siteOrigin2, s1ToO1, s2ToO2, O1ToS1, O2toS2;
		agentOrigin1 = agent1->getPosition(ReferenceFrame::Global);
		agentOrigin2 = agent2->getPosition(ReferenceFrame::Global);
		siteOrigin1 = site1->getPosition(ReferenceFrame::Global);
		siteOrigin2 = site2->getPosition(ReferenceFrame::Global);

		s1ToO1 = (agentOrigin1 - siteOrigin1).normalized();
		O2toS2 = (siteOrigin2 - agentOrigin2).normalized();

		Quaternion rot = Quaternion::FromTwoVectors(O2toS2, s1ToO1);
		agent2->rotateAgent(rot);
		siteOrigin2 = site2->getPosition(ReferenceFrame::Global);

		s2ToO2 = agentOrigin2 - siteOrigin2;
		O1ToS1 = siteOrigin1 - agentOrigin1;
		agent2->setPosition(agentOrigin1 + O1ToS1 + s2ToO2);

		if (!agent1->isInAnyCluster() && !agent2->isInAnyCluster()) {
			simContainer.addAgentCluster(clusterCounter, "default");
			simContainer.addAgentToCluster(agent1->getId(), clusterCounter);
			simContainer.addAgentToCluster(agent2->getId(), clusterCounter);
			simContainer.connectSites(site1, site2);
			//site1->connect(site2);
			clusterCounter++;
		}
		else if(agent1->isInAnyCluster() && !agent2->isInAnyCluster()){
			simContainer.addAgentToCluster(agent2->getId(), agent1->getAgentCluster()->getId());
			//site1->connect(site2);
			simContainer.connectSites(site1, site2);
		}
		else if (!agent1->isInAnyCluster() && agent2->isInAnyCluster()) {
			simContainer.addAgentToCluster(agent1->getId(), agent2->getAgentCluster()->getId());
			simContainer.connectSites(site1, site2);
			//site1->connect(site2);
		}
		else {
			simContainer.addAgentToCluster(agent2->getId(), agent1->getAgentCluster()->getId());
			simContainer.connectSites(site1, site2);
		}
		
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::displayAgent(const unsigned long& id) {
		try {
			vtkVis.renderAxisOfAgentOn = true;
			vtkVis.renderAgent(simContainer.getAgent(id));
			vtkVis.display();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
		
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::displayAgentCluster(const unsigned long& id) {
		try {
			vtkVis.renderAxisOfAgentOn = true;
			vtkVis.renderAxisOfClusterOn = true;
			vtkVis.renderAgentCluster(simContainer.getAgentCluster(id));
			vtkVis.display();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::displayClusterCollisionTree(const unsigned long& clusterId) {
		try {
			vtkVis.renderAxisOfAgentOn = false;
			vtkVis.renderAxisOfClusterOn = true;
			vtkVis.renderEmptyNodesOn = false;
			vtkVis.renderCollisionTree(getAgentCluster(clusterId), collisionDetector.getTree(clusterId));
			vtkVis.display();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API string CLibCollisionController::toString() {
		return simContainer.toString();
	}
}