#pragma once

template <class TClass, class TInterface>
class SimulationObjectFactory {
public:
	static std::shared_ptr<TInterface> Create() { return TClass::CreateInternal(); }
};