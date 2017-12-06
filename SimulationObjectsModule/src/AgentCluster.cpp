#include "..\include\AgentCluster.h"
#include "..\include\Agent.h"

using SimObjPtr = SimulationObject::SimObjPtr;
using std::string;
using unordered_map = std::unordered_map<string, SimObjPtr>;

AgentCluster::AgentCluster()
{
	std::cout << "Creating AgentCluster" << std::endl;
	agents = unordered_map();
}


AgentCluster::~AgentCluster()
{
	std::cout << "Destroying AgentCluster" << std::endl;
}

SimObjPtr AgentCluster::CreateInternal() { 
	//return std::make_shared<AgentCluster>(AgentCluster());
	return SimObjPtr(new AgentCluster());
	//return std::shared_ptr<AgentCluster>(new AgentCluster());
}

void AgentCluster::insertAgent(const string& id, SimObjPtr agent) {
	agents.insert(std::make_pair(id, std::static_pointer_cast<Agent>(agent)));
}

SimObjPtr AgentCluster::getAgent(const string& id) {
	return agents[id];
}

bool AgentCluster::isAgentInCluster(const string& id) {
	return agents.find(id) != agents.end();
}