#pragma once
#include "SimulationObject.h"

class Site :
	public SimulationObject, public SimulationObjectFactory<Site, SimulationObject>
{
public:
	~Site();

	virtual void method1() {};
	static SimObjPtr CreateInternal() { return SimObjPtr(new Site()); }
protected:
	Site();
};

