#include "..\include\TestBench.h"

#include <MetaSpecification.h>
#include <CLibCollisionController.h>
#include <Shape.h>
#include <SimulationObject.h>
#include <Agent.h>

namespace tests {

	TestBench::TestBench(bool showVisualization) : showVisualization(showVisualization) {
		metaSpecs = unordered_map<string, MetaSpecPtr>();
		agentSpecs = unordered_map<string, vector<AgentSpecPtr> >();
		siteSpecs = unordered_map<string, vector<SiteSpecPtr> >();
		shapes = unordered_map<string, vector<ShapePtr> >();
		CLibController = unordered_map<string, CLibPtr>();
	}
}
