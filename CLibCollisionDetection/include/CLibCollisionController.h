#pragma once

#include <SimulationContainer.h>
#include <MetaSpecification.h>
#include <string>
#include <vector>
#include <VTKVisualization.h>

#ifdef CLIBCOLLISIONDETECTION_EXPORTS
#define CLIB_COLLISION_DETECTION_API __declspec(dllexport) 
#else
#define CLIB_COLLISION_DETECTION_API __declspec(dllimport) 
#endif

namespace simobj {
	namespace shapes {
		class Shape;
	}
}

namespace clib
{

	using vis::VTKVisualization;
	using simobj::SimulationContainer;
	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using SiteSpecArray = std::vector<SiteSpecification>;
	using AgentSpecArray = std::vector<AgentSpecification>;
	using std::string;
	using ShapePtr = std::shared_ptr<simobj::shapes::Shape>;

	class CLibCollisionController
	{
	public:

		static CLIB_COLLISION_DETECTION_API SiteSpecification createSiteSpecification(const unsigned long& id, const string& type, const double& x, const double& y, const double& z);

		template<typename... Args>
		static CLIB_COLLISION_DETECTION_API ShapePtr createShape(const unsigned int shapeType, Args... args);

		static CLIB_COLLISION_DETECTION_API AgentSpecification createAgentSpecification(const string& type, ShapePtr shape, SiteSpecArray siteSpecs);

		static CLIB_COLLISION_DETECTION_API MetaSpecification createMetaSpecification(AgentSpecArray agentSpecs);

		CLIB_COLLISION_DETECTION_API CLibCollisionController(const MetaSpecification& metaSpecs);

		CLIB_COLLISION_DETECTION_API void createAgent(const unsigned long& id, const string& type);

		CLIB_COLLISION_DETECTION_API void connectAgents(const unsigned long& agt1, const unsigned long& agt2, const unsigned long& st1, const unsigned long& st2);

		CLIB_COLLISION_DETECTION_API void displayAgent(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API void displayAgentCluster(const unsigned long& id);

		CLIB_COLLISION_DETECTION_API string toString();


	private:
		MetaSpecification metaSpecs;
		SimulationContainer simContainer;
		VTKVisualization vtkVis;
		unsigned long clusterCounter;
	};
}