#include "../include/CLibCollisionController.h"
#include "../include/CLibErrorLogger.h"

#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <OctTree.h>
#include <Shape.h>
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

	CLIB_COLLISION_DETECTION_API SiteSpecification createSiteSpecification(const unsigned long& id, const double& c1, const double& c2, const double& c3, const CoordinateType& cType) {
		return SiteSpecification(id, "default", Vector3d(c1,c2,c3), cType);
	}

	CLIB_COLLISION_DETECTION_API AgentSpecification createAgentSpecification(const string& type, ShapePtr shape, SiteSpecArray siteSpecs) {
		AgentSpecification agent(type, shape);
		for (auto sitespec : siteSpecs) {
			agent.addSiteSpecification(sitespec);
		}
		return agent;
	}

	CLIB_COLLISION_DETECTION_API MetaSpecification createMetaSpecification(AgentSpecArray agentSpecs) {
		MetaSpecification meta;
		for (auto agentspec : agentSpecs) {
			meta.addAgentSpecification(agentspec);
		}
		return meta;
	}

	CLIB_COLLISION_DETECTION_API CLibCollisionController::CLibCollisionController(const double& initialTreeDiameter, const double& minimalCellDiameter, const bool rescalingOn) {
		ErrorLogger::instance();
		simContainer = SimulationContainer();
		collisionDetector = CollisionDetection();
		vtkVis = VTKVisualization();
		nextClusterID = 0;
		collisionDetector.setMinimalCellDiameter(minimalCellDiameter);
		collisionDetector.setInitialTreeDiameter(initialTreeDiameter);
		collisionDetector.setAllowRescaling(rescalingOn);
	}

	CLIB_COLLISION_DETECTION_API CLibCollisionController::CLibCollisionController(const MetaSpecification& metaSpecs, const double& initialTreeDiameter, const double& minimalCellDiameter, const bool rescalingOn) : metaSpecs(metaSpecs) {

		try {
			ErrorLogger::instance();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			std::cout << "Error logging could not be initialized!" << std::endl;
		}

		simContainer = SimulationContainer(metaSpecs);
		collisionDetector = CollisionDetection();
		vtkVis = VTKVisualization();
		nextClusterID = 0;
		collisionDetector.setMinimalCellDiameter(minimalCellDiameter);
		collisionDetector.setInitialTreeDiameter(initialTreeDiameter);
		collisionDetector.setAllowRescaling(rescalingOn);
	}

	CLIB_COLLISION_DETECTION_API CLibCollisionController::~CLibCollisionController() {}

	template<typename... Args>
	CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType& shapeType, Args... args) {
		return ShapeFactory::create(shapeType, args...);
	}

	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double, double);
	template CLIB_COLLISION_DETECTION_API ShapePtr CLibCollisionController::createShape(const ShapeType&, double, double, double);

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

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::setMinimalLeafDiameter(const double& leafDiameter) {
		collisionDetector.setMinimalCellDiameter(leafDiameter);
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::setInitialRootDiameter(const double& rootDiameter) {
		collisionDetector.setInitialTreeDiameter(rootDiameter);
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::addAgentToCollisionDetector(const unsigned long& agentId) {
		shared_ptr<Agent> agent1;
		SimObjPtr clstr;

		try {
			agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agentId));
			clstr = agent1->getAgentCluster();
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}

		if (!collisionDetector.isClusterInTree(clstr->getId())) {
			collisionDetector.makeTreeFromCluster(clstr);
		}
		else {
			try {
				collisionDetector.addAgentToTree(agent1);
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
				LOG_ERROR(e.what());
				return false;
			}
			
		}

		return true;
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::addAgentToCollisionDetector(const unsigned long& agentId, const unsigned long& clusterId) {
		shared_ptr<Agent> agent1;
		shared_ptr<AgentCluster> clstr;
		try {
			agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agentId));
			clstr = std::static_pointer_cast<AgentCluster>(simContainer.getAgentCluster(clusterId));
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		if (agent1->getAgentCluster()->getId() != clstr->getId()) return false;
		return addAgentToCollisionDetector(agentId);
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::addAgentClusterToCollisionDetector(const unsigned long& clusterId) {
		try {
			SimObjPtr clstr = getAgentCluster(clusterId);
			collisionDetector.setAllowRescaling(true);
			collisionDetector.makeTreeFromCluster(clstr);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		return true;
	}

	CLIB_COLLISION_DETECTION_API const bool CLibCollisionController::findNearestToAgent(const unsigned long& agentId, const unsigned long& clusterId, SimObjPtr nearest) {
		shared_ptr<Agent> agent1;
		shared_ptr<AgentCluster> clstr;
		double distance = 0;
		collision::IDSet ignore;

		try {
			agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agentId));
			clstr = std::static_pointer_cast<AgentCluster>(simContainer.getAgentCluster(clusterId));
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return false;
		}
		
		try {
			collisionDetector.checkForCollision(clstr, ignore, agent1, nearest, distance);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			nearest = agent1;
			return false;
		}
		
		if (nullptr == nearest) {
			nearest = agent1;
			return false;
		}
		else {
			return true;
		}
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::checkCollisionBetweenAgents(const unsigned long& agt1, const unsigned long& agt2) {
		if (distanceBetweenAgents(agt1, agt2) < 0) return true;
		return false;
	}

	CLIB_COLLISION_DETECTION_API double CLibCollisionController::distanceBetweenAgents(const unsigned long& agt1, const unsigned long& agt2) {
		shared_ptr<Agent> agent1, agent2;
		try {
			agent1 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt1));
			agent2 = std::static_pointer_cast<Agent>(simContainer.getAgent(agt2));
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return std::numeric_limits<double>::quiet_NaN();
		}

		try {
			return collisionDetector.calcBodyToBodyDistance(agent1, agent2);
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			LOG_ERROR(e.what());
			return std::numeric_limits<double>::quiet_NaN();
		}
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
		agent2->rotate(rot);
		siteOrigin2 = site2->getPosition(ReferenceFrame::Global);

		s2ToO2 = agentOrigin2 - siteOrigin2;
		O1ToS1 = siteOrigin1 - agentOrigin1;
		agent2->setPosition(agentOrigin1 + O1ToS1 + s2ToO2);

		if (!agent1->isInAnyCluster() && !agent2->isInAnyCluster()) {
			simContainer.addAgentCluster(generateNextClusterID(), "default");
			SimObjPtr cluster = simContainer.getAgentCluster(nextClusterID);
			simContainer.addAgentToCluster(agent1, cluster);
			simContainer.addAgentToCluster(agent2, cluster);
			simContainer.connectAgents(agent1, agent2, site1->getId(), site2->getId());
		}
		else if(agent1->isInAnyCluster() && !agent2->isInAnyCluster()){
			simContainer.addAgentToCluster(agent2, agent1->getAgentCluster());
			simContainer.connectAgents(agent1, agent2, site1->getId(), site2->getId());
		}
		else if (!agent1->isInAnyCluster() && agent2->isInAnyCluster()) {
			simContainer.addAgentToCluster(agent1, agent2->getAgentCluster());
			simContainer.connectAgents(agent1, agent2, site1->getId(), site2->getId());
		}
		else {
			simContainer.addAgentToCluster(agent2, agent1->getAgentCluster());
			simContainer.connectAgents(agent1, agent2, site1->getId(), site2->getId());
		}
		
	}

	CLIB_COLLISION_DETECTION_API bool CLibCollisionController::displayAgent(const unsigned long& id) {
		try {
			vtkVis.renderAxisOfAgentOn = true;
			vtkVis.renderGlobalAxisOn = false;
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
			vtkVis.renderAxisOfAgentOn = false;
			vtkVis.renderGlobalAxisOn = false;
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
			vtkVis.renderEmptyNodesOn = true;
			vtkVis.renderGlobalAxisOn = false;
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

	CLIB_COLLISION_DETECTION_API const unsigned long CLibCollisionController::generateNextClusterID() {
		while (simContainer.isAgentClusterInContainer(nextClusterID)) {
			nextClusterID++;
		}
		return nextClusterID;
	}
}