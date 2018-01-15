#include "..\include\SimulationContainer.h"
#include "..\include\SimulationObject.h"
#include "..\include\Agent.h"
#include "..\include\Site.h"
#include "..\include\AgentCluster.h"

namespace simobj {

	SimulationContainer::SimulationContainer() {
		agents = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		clusters = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		metaSpecs = MetaSpecification();
	}

	SimulationContainer::SimulationContainer(const MetaSpecification& metaSpecs)
	{
		agents = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		clusters = std::unordered_map<unsigned long, SimulationObject::SimObjPtr>();
		this->metaSpecs = metaSpecs;
	}

	void SimulationContainer::addAgent(const unsigned long& id, const string& type) {
		if (isAgentInContainer(id)) throw std::runtime_error("Agent with given ID already present within SimulationContainer!");
		agents.insert(std::make_pair(id, Agent::create(id, metaSpecs.getAgentSpecification(type))));
	}

	void SimulationContainer::addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId) {
		shared_ptr<Agent> agent = std::static_pointer_cast<Agent>(getAgent(agentId));
		shared_ptr<AgentCluster> cluster = std::static_pointer_cast<AgentCluster>(getAgentCluster(clusterId));
		cluster->insertAgent(agent);
		agent->setAgentCluster(cluster);
	}

	void SimulationContainer::addAgentCluster(const unsigned long& id, const string& type) {
		if (isAgentClusterInContainer(id)) throw std::runtime_error("AgentCluster with given ID already present within SimulationContainer!");
		clusters.insert(std::make_pair(id, AgentCluster::create(id, type)));
	}

	void SimulationContainer::connectSites(SimObjPtr site1, SimObjPtr site2) {
		shared_ptr<Site> st1 = std::static_pointer_cast<Site>(site1);
		shared_ptr<Site> st2 = std::static_pointer_cast<Site>(site2);
		st1->connect(st2);
		st2->connect(st1);
	}

	SimObjPtr SimulationContainer::getAgent(const unsigned long& id) {
		if (!isAgentInContainer(id)) throw std::runtime_error("Agent with given ID not present within SimulationContainer!");
		return agents.at(id);
	}

	SimObjPtr SimulationContainer::getAgentCluster(const unsigned long& id) {
		if (!isAgentClusterInContainer(id)) throw std::runtime_error("AgentCluster with given ID not present within SimulationContainer!");
		return clusters.at(id);
	}

	const bool SimulationContainer::isAgentInContainer(const unsigned long& id) const {
		return agents.find(id) != agents.end();
	}

	const bool SimulationContainer::isAgentClusterInContainer(const unsigned long& id) const {
		return clusters.find(id) != clusters.end();
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