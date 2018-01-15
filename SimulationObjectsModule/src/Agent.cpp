#include "..\include\Agent.h"
#include "..\include\Shape.h"
#include "..\include\Site.h"
#include "..\include\AgentCluster.h"
#include <sstream>

namespace simobj {

	Agent::Agent(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		sites = SitesMap();
		belongsToCluster = false;
		hasShape = false;
		cluster = SimObjWeakPtr();
		shape = ShapePtr();
	}

	Agent::~Agent()
	{
	}

	string Agent::toString() const {
		std::stringstream ss;
		ss << "** Agent: [" << " Type: " << type << ", ID: " << id << ",\n";
		ss << "\t +++ Position: x: " << position.x() << ", y: " << position.y() << ", z: " << position.z() << ", \n";
		ss << "\t +++ Orienation: qw: " << orientation.w() << ", qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", \n";
		if (hasShape) {
			ss << "\t +++ Shape: " << shape->toString() << ", \n";
		}
		ss << "\t +++ belongs to a cluster: " << ((belongsToCluster) ? "true" : "false") << ", \n";
		if (belongsToCluster) {
			ss << "\t +++ Cluster info : connected to cluster-id: " << cluster.lock()->getId() << ", cluster-type: " << cluster.lock()->getType() << ", \n";
		}
		ss << "\t +++ Attached sites: \n";
		for (auto site : sites) {
			ss << site.second->toString();
		}
		ss << "] ** \n";
		return ss.str();
	}

	const Vector3d Agent::getPosition(const ReferenceFrame& frame) const {
		switch (frame) {
			case ReferenceFrame::Local : {
				return position;
				break;
			}
			case ReferenceFrame::Global: {
				if (belongsToCluster) {
					return cluster.lock()->getPosition() + cluster.lock()->getOrientation()*position;
				}
				else {
					return position;
				}
				break;
			}
			default: {
				throw std::runtime_error("Given reference frame does not exist or is unsupported!");
			}
		}
	}

	const Quaternion Agent::getOrientation(const ReferenceFrame& frame) const {
		switch (frame) {
		case ReferenceFrame::Local: {
			return orientation;
			break;
		}
		case ReferenceFrame::Global: {
			if (belongsToCluster) {
				return cluster.lock()->getOrientation()*orientation;
			}
			else {
				return orientation;
			}
			break;
		}
		default: {
			throw std::runtime_error("Given reference frame does not exist or is unsupported!");
		}
		}
	}

	SimObjPtr Agent::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new Agent(id, type));
	}

	void Agent::addSite(SimObjPtr site) {
		SimObjPtr s = std::static_pointer_cast<Site>(site);
		if (isSiteAtAgent(s->getId())) throw std::runtime_error("Site with given ID already present at this Agent!");
		sites.insert(std::make_pair(s->getId(), s));
	}

	void Agent::setAgentCluster(SimObjPtr cluster) {
		SimObjPtr clstr = std::static_pointer_cast<AgentCluster>(cluster);
		this->cluster = cluster;
		belongsToCluster = true;
	}

	void Agent::setShape(ShapePtr shape) {
		this->shape = shape;
		hasShape = true;
	}

	SimObjPtr Agent::getSite(const unsigned long& id) {
		if (!isSiteAtAgent(id)) throw std::runtime_error("Site with given ID not present at this Agent!");
		return sites.at(id);
	}

	const SitesMap& Agent::getAllSites() const {
		return sites;
	}

	ShapePtr Agent::getShape() {
		if (!hasShape) throw std::runtime_error("This agent does not posses any defined shape!");
		return shape;
	}

	SimObjPtr Agent::getAgentCluster() {
		return cluster.lock();
	}

	void Agent::rotateAgent(const Quaternion& rotation) {
		orientation = orientation*rotation;
	}

	void Agent::moveAgent(const Vector3d& translation) {
		position = position + translation;
	}

	bool Agent::isInAnyCluster() const {
		return belongsToCluster;
	}

	bool Agent::isAgentCluster(SimObjPtr cluster) const {
		return this->cluster.lock()->getId() == cluster->getId();
	}

	bool Agent::isSiteAtAgent(const unsigned long& id) const {
		return sites.find(id) != sites.end();
	}
}