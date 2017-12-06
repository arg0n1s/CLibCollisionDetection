#pragma once

#include <unordered_map>
#include <memory>

class SimulationObject;

using SimObjPtr = std::shared_ptr<SimulationObject>;
using std::string;
using unordered_map = std::unordered_map<string, SimObjPtr>;

class SimulationContainer
{
public:
	SimulationContainer(const unsigned int numOfAgents);
	void addAgent(const string& id);
	void addAgentCluster(const string& id);
private:
	unordered_map agents;
	unordered_map clusters;
};
