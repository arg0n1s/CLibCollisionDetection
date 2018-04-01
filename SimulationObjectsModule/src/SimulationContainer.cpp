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
		agents.insert(std::make_pair(id, Agent::NewAgentFromSpecification(id, metaSpecs.getAgentSpecification(type))));
	}

	void SimulationContainer::addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId) {
		addAgentToCluster(getAgent(agentId), getAgentCluster(clusterId));
	}

	void SimulationContainer::addAgentToCluster(SimObjPtr agent, SimObjPtr cluster) {
		shared_ptr<Agent> agentPtr = std::static_pointer_cast<Agent>(agent);
		shared_ptr<AgentCluster> clusterPtr = std::static_pointer_cast<AgentCluster>(cluster);
		clusterPtr->insertAgent(agentPtr);
		agentPtr->setAgentCluster(clusterPtr);
	}

	void SimulationContainer::addAgentCluster(const unsigned long& id, const string& type) {
		if (isAgentClusterInContainer(id)) throw std::runtime_error("AgentCluster with given ID already present within SimulationContainer!");
		clusters.insert(std::make_pair(id, AgentCluster::New(id, type)));
	}

	void SimulationContainer::connectAgents(const unsigned long& agent1, const unsigned long& agent2, const unsigned long& site1, const unsigned long& site2) {
		connectAgents(getAgent(agent1), getAgent(agent2), site1, site2);
	}

	void SimulationContainer::connectAgents(SimObjPtr agent1, SimObjPtr agent2, const unsigned long& site1, const unsigned long& site2) {
		shared_ptr<Agent> agentPtr1 = std::static_pointer_cast<Agent>(agent1);
		shared_ptr<Agent> agentPtr2 = std::static_pointer_cast<Agent>(agent2);
		connectSites(agentPtr1->getSite(site1), agentPtr2->getSite(site2));
	}

	void SimulationContainer::connectSites(SimObjPtr site1, SimObjPtr site2) {
		shared_ptr<Site> st1 = std::static_pointer_cast<Site>(site1);
		shared_ptr<Site> st2 = std::static_pointer_cast<Site>(site2);
		if(st1->getOwner()->getId() == st2->getOwner()->getId()) throw std::runtime_error("Connecting two Sites of the same Agent is not allowed!");
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
		ss << "*** \nSimulation Object Container: [";
		ss << "\tContained agents {" << agents.size() << "}: (";
		for (auto agent : agents) {
			ss << agent.second->getId() << ", ";
		}
		ss << ")\n";
		ss << "\t\t\t\tContained AgentClusters {" << clusters.size() << "}: (";
		for (auto cluster : clusters) {
			ss << cluster.second->getId() << ", ";
		}
		ss << ")";
		ss << " ] \n***";
		return ss.str();
	}
}