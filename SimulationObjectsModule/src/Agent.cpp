#include "..\include\Agent.h"
#include "..\include\Shape.h"
#include "..\include\Site.h"

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