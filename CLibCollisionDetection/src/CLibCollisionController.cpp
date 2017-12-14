#include "../include/CLibCollisionController.h"
#include <Eigen\Core>
#include <Eigen\Dense>
#include <Shape.h>

namespace clib {

	using Eigen::Vector3d;
	using simobj::shapes::ShapeFactory;
	using simobj::shapes::ShapePtr;
	using simobj::shapes::ShapeType;

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
	}

	CLIB_COLLISION_DETECTION_API void CLibCollisionController::createAgent(const unsigned long& id, const string& type) {
		simContainer.addAgent(id, type);
	}

	CLIB_COLLISION_DETECTION_API string CLibCollisionController::toString() {
		return simContainer.toString();
	}
}