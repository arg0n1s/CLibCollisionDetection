#pragma once
#include "SimulationObject.h"

class Agent :
	public SimulationObject, public SimulationObjectFactory<Agent, SimulationObject>
{
public:
	~Agent();

	virtual void method1() {};
	static SimObjPtr CreateInternal() { return SimObjPtr(new Agent()); }
protected:
	Agent();
};

