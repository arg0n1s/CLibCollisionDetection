#pragma once
#include "SimulationObject.h"

class Molecule :
	public SimulationObject, public SimulationObjectFactory<Molecule, SimulationObject>
{
public:
	~Molecule();

	virtual void method1() {};
	static SimObjPtr CreateInternal() { return SimObjPtr(new Molecule()); }
protected:
	Molecule();
};