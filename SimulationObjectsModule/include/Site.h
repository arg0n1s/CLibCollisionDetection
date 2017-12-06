#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"

class Site :
	public SimulationObject, public SimulationObjectFactory<Site, SimulationObject>
{
public:
	~Site();

	virtual void method1() {}
	static SimObjPtr CreateInternal(SimObjPtr owner);

	bool isConnected() const;
	void connect(SimObjPtr otherSite);
	SimObjPtr getOwnerAgent();

protected:
	Site(SimObjPtr owner);
	bool connected;
	SimObjPtr ownerAgent;
	SimObjPtr otherSite;
};

