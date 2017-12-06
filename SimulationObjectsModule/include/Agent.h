#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"

using std::string;

class Agent :
	public SimulationObject, public SimulationObjectFactory<Agent, SimulationObject>
{
public:
	~Agent();

	virtual void method1() {};
	static SimObjPtr CreateInternal();

	void setAgentClusterId(const string& id);
	const string& getAgentClusterId() const;
	bool isInAnyCluster() const;
	bool isAgentCluster(const string& clusterId) const;
protected:
	Agent();
	string clusterId;
	bool belongsToCluster;
};

