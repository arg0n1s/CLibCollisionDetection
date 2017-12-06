#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"
#include <unordered_map>

using SimObjPtr = SimulationObject::SimObjPtr;
using std::string;
using unordered_map = std::unordered_map<string, SimObjPtr>;


class AgentCluster :
	public SimulationObject, public SimulationObjectFactory<AgentCluster, SimulationObject>
{
public:
	~AgentCluster();

	virtual void method1() {};
	static SimObjPtr CreateInternal();

	void insertAgent(const string& id, SimObjPtr agent);
	SimObjPtr getAgent(const string& id);
	bool isAgentInCluster(const string& id);

protected:
	AgentCluster();
	unordered_map agents;
};