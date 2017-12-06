#include "..\include\SimulationContainer.h"
#include "..\include\Agent.h"
#include "..\include\Molecule.h"


SimulationContainer::SimulationContainer(const unsigned int numOfAgents)
{
	agents = std::unordered_map<std::string, SimulationObject::SimObjPtr>(numOfAgents);
	molecules = std::unordered_map<std::string, SimulationObject::SimObjPtr>();
}

void SimulationContainer::addAgent(const std::string& id) {
	agents.insert(std::make_pair(id, Agent::Create()));
}

void SimulationContainer::addMolecule(const std::string& id) {
	molecules.insert(std::make_pair(id, Molecule::Create()));
}