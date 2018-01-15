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
		SimulationContainer();
		SimulationContainer(const MetaSpecification& metaSpecs);
		void addAgent(const unsigned long& id, const string& type);
		void addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId);
		void addAgentCluster(const unsigned long& id, const string& type);

		SimObjPtr getAgent(const unsigned long& id);
		SimObjPtr getAgentCluster(const unsigned long& id);

		const bool isAgentInContainer(const unsigned long& id) const;
		const bool isAgentClusterInContainer(const unsigned long& id) const;

		string toString() const;
	private:
		unordered_map agents;
		unordered_map clusters;
		MetaSpecification metaSpecs;
	};
}
