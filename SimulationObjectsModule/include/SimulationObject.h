#pragma once
//#include <boost\shared_ptr.hpp>
#include <iostream>
#include <memory>

template <class TClass, class TInterface>
class SimulationObjectFactory {
public:
	static std::shared_ptr<TInterface> Create() { return TClass::CreateInternal(); }
};

class SimulationObject {
public:
	typedef std::shared_ptr<SimulationObject> SimObjPtr;

	SimulationObject();
	virtual ~SimulationObject();
	virtual void method1() = 0;
};