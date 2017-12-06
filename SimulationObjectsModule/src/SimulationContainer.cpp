#include "..\include\SimulationContainer.h"
#include "..\include\SimulationObject.h"
#include "..\include\Agent.h"
#include "..\include\AgentCluster.h"


SimulationContainer::SimulationContainer(const unsigned int numOfAgents)
{
	agents = std::unordered_map<std::string, SimulationObject::SimObjPtr>(numOfAgents);
	clusters = std::unordered_map<std::string, SimulationObject::SimObjPtr>();
}

void SimulationContainer::addAgent(const std::string& id) {
	agents.insert(std::make_pair(id, Agent::Create()));
}

void SimulationContainer::addAgentCluster(const std::string& id) {
	clusters.insert(std::make_pair(id, AgentCluster::Create()));
}