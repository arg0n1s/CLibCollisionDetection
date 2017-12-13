#include "..\include\AgentCluster.h"
#include "..\include\Agent.h"

namespace simobj {
	using SimObjPtr = SimulationObject::SimObjPtr;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;

	AgentCluster::AgentCluster(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		agents = unordered_map();
	}


	AgentCluster::~AgentCluster()
	{
	}

	SimObjPtr AgentCluster::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new AgentCluster(id, type));
	}

	void AgentCluster::insertAgent(SimObjPtr agent) {
		agents.insert(std::make_pair(agent->getId(), std::static_pointer_cast<Agent>(agent)));
	}

	SimObjPtr AgentCluster::getAgent(const unsigned long& id) {
		return agents[id];
	}

	bool AgentCluster::isAgentInCluster(const unsigned long& id) {
		return agents.find(id) != agents.end();
	}
}