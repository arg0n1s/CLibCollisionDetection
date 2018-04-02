#pragma once
#include <vector>
#include <unordered_map>
#include <memory>
#include <string>

namespace clib {
	class CLibCollisionController;
}

namespace simobj {

	namespace specs {
		class MetaSpecification;
		class AgentSpecification;
		class SiteSpecification;
	}

	namespace shapes {
		class Shape;
	}

	class SimulationObject;
}

namespace tests {
	using std::shared_ptr;
	using std::vector;
	using std::unordered_map;
	using std::string;

	using MetaSpecPtr = shared_ptr<simobj::specs::MetaSpecification>;
	using AgentSpecPtr = shared_ptr<simobj::specs::AgentSpecification>;
	using SiteSpecPtr = shared_ptr<simobj::specs::SiteSpecification>;
	using ShapePtr = shared_ptr<simobj::shapes::Shape>;
	using SimObjPtr = shared_ptr<simobj::SimulationObject>;
	using CLibPtr = shared_ptr<clib::CLibCollisionController>;

	class TestBench {

	public:
		TestBench(bool showVisualization = false);
		virtual void setup() = 0;
		virtual void runAllTests() = 0;

	protected:
		unordered_map<string, CLibPtr> CLibController;
		unordered_map<string, MetaSpecPtr> metaSpecs;
		unordered_map<string, vector<AgentSpecPtr> > agentSpecs;
		unordered_map<string, vector<SiteSpecPtr> > siteSpecs;
		unordered_map<string, vector<ShapePtr > > shapes;

		bool showVisualization;
	};

}
