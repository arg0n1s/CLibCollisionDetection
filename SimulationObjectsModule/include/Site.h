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
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void setOwner(SimObjPtr owner);
		SimObjWeakPtr getOwner();
		bool isConnected() const;
		void connect(SimObjPtr otherSite);

	protected:
		Site(const unsigned long& id, const string& type);
		bool connected;
		bool hasOwner;
		SimObjWeakPtr ownerAgent;
		SimObjWeakPtr otherSite;
	};
}

