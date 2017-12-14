#include "..\include\SimulationContainer.h"
#include "..\include\SimulationObject.h"
#include "..\include\Agent.h"
#include "..\include\AgentCluster.h"

namespace simobj {

	SimulationContainer::SimulationContainer(const MetaSpecification& metaSpecs)
	{
		agents = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		clusters = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		this->metaSpecs = metaSpecs;
	}

	void SimulationContainer::addAgent(const unsigned long& id, const string& type) {
		if (!metaSpecs.isAgentInSpecs(type)) return;
		agents.insert(std::make_pair(id, Agent::create(id, metaSpecs.getAgentSpecification(type))));
	}

	void SimulationContainer::addAgentCluster(const unsigned long& id, const string& type) {
		clusters.insert(std::make_pair(id, AgentCluster::create(id, type)));
	}

	string SimulationContainer::toString() const {
		std::stringstream ss;
		ss << "Simulation Object Container: [ \n";
		ss << "********** Contained Agents: \n";
		for (auto agent : agents) {
			ss << agent.second->toString();
		}
		ss << "********** Contained AgentClusters: \n";
		for (auto cluster : clusters) {
			ss << cluster.second->toString();
		}
		ss << "]";
		return ss.str();
	}
}