#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"

namespace simobj {

	class Site :
		public SimulationObject, public SimulationObjectFactory<Site, SimulationObject>
	{
	public:
		~Site();

		virtual string toString() const;
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void setOwner(SimObjPtr owner);
		SimObjPtr getOwner();
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

