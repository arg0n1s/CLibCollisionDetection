#include "..\include\Site.h"
#include "..\include\Agent.h"
#include <sstream>

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

	string Site::toString() const {
		std::stringstream ss;
		ss << "Site: [" << " Type: " << type << ", ID: " << id << ",\n";
		ss << "\t x: " << position.x() << ", y: " << position.y() << ", z: " << position.z() << ", \n";
		ss << "\t connected: " << ((connected) ? "true" : "false") << ", \n";
		ss << "\t owner-id: " << ownerAgent->getId() << ", owner-type: " << ownerAgent->getType() << ", \n";
		if (connected) {
			shared_ptr<Site> other = std::static_pointer_cast<Site>(otherSite);
			ss << "\t connected to site-id: " << other->getId() << ", site-type: " << other->getType() << ", \n";
			ss << "\t connected to agent-id: " << other->getOwner()->getId() << " agent-type: " << other->getOwner()->getId() << ", \n";
		}
		ss << "] \n";
		return ss.str();
	}

	SimObjPtr Site::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new Site(id, type));
	}

	void Site::setOwner(SimObjPtr owner) {
		this->ownerAgent = std::static_pointer_cast<Agent>(owner);
	}

	SimObjPtr Site::getOwner() {
		return ownerAgent;
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

