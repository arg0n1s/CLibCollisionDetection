#pragma once

#include <SimulationContainer.h>
#include <MetaSpecification.h>
#include <CollisionDetection.h>
#include <string>
#include <vector>
#include <Shape.h>
#include <VTKVisualization.h>

#ifdef CLIBCOLLISIONDETECTION_EXPORTS
#define CLIB_COLLISION_DETECTION_API __declspec(dllexport) 
#else
#define CLIB_COLLISION_DETECTION_API __declspec(dllimport) 
#endif

namespace clib
{

	using vis::VTKVisualization;
	using simobj::SimulationContainer;
	using simobj::SimObjPtr;
	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using simobj::specs::CoordinateType;
	using SiteSpecArray = std::vector<SiteSpecification>;
	using AgentSpecArray = std::vector<AgentSpecification>;
	using std::string;
	using ShapePtr = std::shared_ptr<simobj::shapes::Shape>;
	using simobj::shapes::ShapeType;
	using collision::CollisionDetection;

	class CLibCollisionController
	{
	public:

		static CLIB_COLLISION_DETECTION_API SiteSpecification createSiteSpecification(const unsigned long& id, const double& c1, const double& c2, const double& c3, const CoordinateType& cType);

		template<typename... Args>
		static CLIB_COLLISION_DETECTION_API ShapePtr createShape(const ShapeType& shapeType, Args... args);

		static CLIB_COLLISION_DETECTION_API AgentSpecification createAgentSpecification(const string& type, ShapePtr shape, SiteSpecArray siteSpecs);

		static CLIB_COLLISION_DETECTION_API MetaSpecification createMetaSpecification(AgentSpecArray agentSpecs);

		CLIB_COLLISION_DETECTION_API CLibCollisionController(const MetaSpecification& metaSpecs);

		CLIB_COLLISION_DETECTION_API bool createAgentCluster(const unsigned long& id, const string& type);

		CLIB_COLLISION_DETECTION_API bool createAgent(const unsigned long& id, const string& type);

		CLIB_COLLISION_DETECTION_API SimObjPtr getAgent(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API SimObjPtr getAgentCluster(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API bool addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId);

		CLIB_COLLISION_DETECTION_API bool addAgentClusterToCollisionDetector(const unsigned long& clusterId);

		CLIB_COLLISION_DETECTION_API void connectAgents(const unsigned long& agt1, const unsigned long& agt2, const unsigned long& st1, const unsigned long& st2);

		CLIB_COLLISION_DETECTION_API bool displayAgent(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API bool displayAgentCluster(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API bool displayClusterCollisionTree(const unsigned long& clusterId);

		CLIB_COLLISION_DETECTION_API string toString();


	private:
		MetaSpecification metaSpecs;
		SimulationContainer simContainer;
		CollisionDetection collisionDetector;
		VTKVisualization vtkVis;
		unsigned long clusterCounter;
	};
}