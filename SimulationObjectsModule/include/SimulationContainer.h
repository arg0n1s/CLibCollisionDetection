#pragma once

#include <unordered_map>
#include <memory>
#include "../include/MetaSpecification.h"

namespace simobj {

	class SimulationObject;

	using SimObjPtr = std::shared_ptr<SimulationObject>;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;
	using specs::MetaSpecification;

	class SimulationContainer
	{
	public:
		SimulationContainer(const MetaSpecification& metaSpecs);
		void addAgent(const unsigned long& id, const string& type);
		void addAgentCluster(const unsigned long& id, const string& type);
	private:
		unordered_map agents;
		unordered_map clusters;
		MetaSpecification metaSpecs;
	};
}
