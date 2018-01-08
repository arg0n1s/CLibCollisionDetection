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
		hasOwner = false;
	}


	Site::~Site()
	{
	}

	string Site::toString() const {
		std::stringstream ss;
		ss << "Site: [" << " Type: " << type << ", ID: " << id << ",\n";
		ss << "\t x: " << position.x() << ", y: " << position.y() << ", z: " << position.z() << ", \n";
		ss << "\t connected: " << ((connected) ? "true" : "false") << ", \n";
		if (hasOwner) {
			ss << "\t owner-id: " << ownerAgent->getId() << ", owner-type: " << ownerAgent->getType() << ", \n";
		}
		if (connected) {
			shared_ptr<Site> other = std::static_pointer_cast<Site>(otherSite);
			ss << "\t connected to site-id: " << other->getId() << ", site-type: " << other->getType() << ", \n";
			ss << "\t connected to agent-id: " << other->getOwner()->getId() << " agent-type: " << other->getOwner()->getId() << ", \n";
		}
		ss << "] \n";
		return ss.str();
	}

	const Vector3d Site::getPosition(const ReferenceFrame& frame) const {
		switch (frame) {
		case ReferenceFrame::Local: {
			return position;
			break;
		}
		case ReferenceFrame::Global: {
			if (!hasOwner) throw std::runtime_error("This Site does not seem to belong to any known agent! \n No known reference frame found.");
			return ownerAgent->getPosition(frame) + ownerAgent->getOrientation()*position;
			break;
		}
		default: {
			throw std::runtime_error("Given reference frame does not exist or is unsupported!");
		}
		}
	}
	const Quaternion Site::getOrientation(const ReferenceFrame& frame) const {
		switch (frame) {
		case ReferenceFrame::Local: {
			return orientation;
			break;
		}
		case ReferenceFrame::Global: {
			if (!hasOwner) throw std::runtime_error("This Site does not seem to belong to any known agent! \n No known reference frame found.");
			return ownerAgent->getOrientation(frame) * orientation;
			break;
		}
		default: {
			throw std::runtime_error("Given reference frame does not exist or is unsupported!");
		}
		}
	}

	SimObjPtr Site::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new Site(id, type));
	}

	void Site::setOwner(SimObjPtr owner) {
		this->ownerAgent = std::static_pointer_cast<Agent>(owner);
		hasOwner = true;
	}

	SimObjPtr Site::getOwner() {
		if (!hasOwner) throw std::runtime_error("This Site does not seem to belong to any known agent!");
		return ownerAgent;
	}

	bool Site::isConnected() const {
		return connected;
	}

	void Site::connect(SimObjPtr otherSite) {
		this->otherSite = std::static_pointer_cast<Site>(otherSite);
		connected = true;
	}

}

