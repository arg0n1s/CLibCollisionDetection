#include "..\include\Agent.h"
#include "..\include\Shape.h"
#include "..\include\Site.h"
#include <sstream>

namespace simobj {

	Agent::Agent(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		sites = SitesMap();
		belongsToCluster = false;
		cluster = SimObjPtr();
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
		ss << "\t +++ Shape: " << shape->toString() << ", \n";
		ss << "\t +++ belongs to a cluster: " << ((belongsToCluster) ? "true" : "false") << ", \n";
		if (belongsToCluster) {
			ss << "\t +++ Cluster info : connected to cluster-id: " << cluster->getId() << ", cluster-type: " << cluster->getType() << ", \n";
		}
		ss << "\t +++ Attached sites: \n";
		for (auto site : sites) {
			ss << site.second->toString();
		}
		ss << "] ** \n";
		return ss.str();
	}

	SimObjPtr Agent::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new Agent(id, type));
	}

	void Agent::addSite(SimObjPtr site) {
		SimObjPtr s = std::static_pointer_cast<Site>(site);
		sites.insert(std::make_pair(s->getId(), s));
	}

	void Agent::setAgentCluster(SimObjPtr cluster) {
		this->cluster = cluster;
		belongsToCluster = true;
	}

	void Agent::setShape(ShapePtr shape) {
		this->shape = shape;
	}

	ShapePtr Agent::getShape() {
		return shape;
	}

	SimObjPtr Agent::getAgentCluster() {
		return cluster;
	}

	bool Agent::isInAnyCluster() const {
		return belongsToCluster;
	}

	bool Agent::isAgentCluster(SimObjPtr cluster) const {
		return this->cluster->getId() == cluster->getId();
	}
}