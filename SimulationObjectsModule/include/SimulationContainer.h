#pragma once

#include "SimulationObject.h"
#include <unordered_map>

class SimulationContainer
{
public:
	SimulationContainer(const unsigned int numOfAgents);
	void addAgent(const std::string& id);
	void addMolecule(const std::string& id);
private:
	std::unordered_map<std::string, SimulationObject::SimObjPtr> agents;
	std::unordered_map<std::string, SimulationObject::SimObjPtr> molecules;
};
