#include "..\include\Agent.h"

using SimObjPtr = SimulationObject::SimObjPtr;

Agent::Agent()
{
	std::cout << "Creating Agent" << std::endl;
	belongsToCluster = false;
	clusterId = "";
}


Agent::~Agent()
{
	std::cout << "Destroying Agent" << std::endl;
}

SimObjPtr Agent::CreateInternal() { 
	//return std::shared_ptr<Agent>(new Agent());
	//return std::make_shared<Agent>(Agent()); 
	return SimObjPtr(new Agent());
}

void Agent::setAgentClusterId(const string& id) {
	clusterId = id;
}
const string& Agent::getAgentClusterId() const {
	return clusterId;
}
bool Agent::isInAnyCluster() const {
	return belongsToCluster;
}
bool Agent::isAgentCluster(const string& clusterId) const {
	return this->clusterId.compare(clusterId) == 0;
}