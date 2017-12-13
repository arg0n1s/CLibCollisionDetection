#include "..\include\Site.h"
#include "..\include\Agent.h"

namespace simobj {

	using SimObjPtr = SimulationObject::SimObjPtr;

	Site::Site(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		ownerAgent = std::shared_ptr<Agent>();
		connected = false;
		otherSite = std::shared_ptr<Site>();
	}


	Site::~Site()
	{
	}

	SimObjPtr Site::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new Site(id, type));
	}

	void Site::setOwner(SimObjPtr owner) {
		this->ownerAgent = std::static_pointer_cast<Agent>(owner);
	}

	bool Site::isConnected() const {
		return connected;
	}

	void Site::connect(SimObjPtr otherSite) {
		this->otherSite = std::static_pointer_cast<Site>(otherSite);
		connected = true;
	}

	SimObjPtr Site::getOwnerAgent() {
		return ownerAgent;
	}
}

