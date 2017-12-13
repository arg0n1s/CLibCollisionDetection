#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"

namespace simobj {

	class Site :
		public SimulationObject, public SimulationObjectFactory<Site, SimulationObject>
	{
	public:
		~Site();

		virtual void method1() {}
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void setOwner(SimObjPtr owner);
		bool isConnected() const;
		void connect(SimObjPtr otherSite);
		SimObjPtr getOwnerAgent();

	protected:
		Site(const unsigned long& id, const string& type);
		bool connected;
		SimObjPtr ownerAgent;
		SimObjPtr otherSite;
	};
}

