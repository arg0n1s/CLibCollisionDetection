#include "..\include\Site.h"
#include "..\include\Agent.h"
#include <sstream>

namespace simobj {

	using SimObjPtr = SimulationObject::SimObjPtr;
	using SimObjWeakPtr = SimulationObject::SimObjWeakPtr;

	Site::Site(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		ownerAgent = SimObjWeakPtr();
		connected = false;
		otherSite = SimObjWeakPtr();
		hasOwner = false;
	}


	Site::~Site()
	{
	}

	string Site::toString() const {
		std::stringstream ss;
		ss << "*** \nSite: [" << " Type: " << type << ", ID: " << id << ",\n";
		ss << "\tPosition: x: " << position.x() << ", y: " << position.y() << ", z: " << position.z() << ",\n";
		ss << "\tconnected: " << ((connected) ? "true" : "false") << ",\n";
		if (hasOwner) {
			ss << "\towner-id: " << ownerAgent.lock()->getId() << ", owner-type: " << ownerAgent.lock()->getType();
		}
		if (connected) {
			shared_ptr<Site> other = std::static_pointer_cast<Site>(otherSite.lock());
			ss << "\n\tconnected to site-id: " << other->getId() << ", site-type: " << other->getType() << ",\n";
			ss << "\tconnected to agent-id: " << other->getOwner()->getId() << " agent-type: " << other->getOwner()->getId();
		}
		ss << " ] \n***";
		return ss.str();
	}

	const Vector3d& Site::getPosition() const {
		return position;
	}

	const Quaternion& Site::getOrientation() const {
		return orientation;
	}

	const Vector3d Site::getPosition(const ReferenceFrame& frame) const {
		switch (frame) {
		case ReferenceFrame::Local: {
			return position;
		}
		case ReferenceFrame::Global: {
			if (hasOwner) {
				return ownerAgent.lock()->getPosition(frame) + ownerAgent.lock()->getOrientation(frame)*position;
			}
			else {
				return position;
			}
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
		}
		case ReferenceFrame::Global: {
			if (hasOwner) {
				return ownerAgent.lock()->getOrientation(frame) * orientation;
			}
			else {
				return orientation;
			}
		}
		default: {
			throw std::runtime_error("Given reference frame does not exist or is unsupported!");
		}
		}
	}

	SimObjPtr Site::New(const unsigned long& id, const string& type) {
		return SimObjPtr(new Site(id, type));
	}

	void Site::setOwner(SimObjPtr owner) {
		this->ownerAgent = std::static_pointer_cast<Agent>(owner);
		hasOwner = true;
	}

	SimObjPtr Site::getOwner() {
		if (!hasOwner) throw std::runtime_error("This Site does not seem to belong to any known agent!");
		return ownerAgent.lock();
	}

	bool Site::isConnected() const {
		return connected;
	}

	void Site::connect(SimObjPtr otherSite) {
		this->otherSite = std::static_pointer_cast<Site>(otherSite);
		connected = true;
	}

}

