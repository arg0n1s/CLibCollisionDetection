#include "../include/SimulationObjectFactory.h"
#include "../include/MetaSpecification.h"
#include "../include/SimulationObject.h"
#include "../include/Agent.h"
#include "../include/Site.h"
#include "../include/AgentCluster.h"

namespace simobj {

	template <typename TClass, typename TInterface>
	shared_ptr<TInterface> SimulationObjectFactory<TClass, TInterface>::create(const unsigned long& id, const string& type) { 
		return TClass::createInternal(id, type); 
	}
	template <typename TClass, typename TInterface>
	shared_ptr<TInterface> SimulationObjectFactory<TClass, TInterface>::create(const unsigned long& id, const AgentSpecification& agentSpec) {
		shared_ptr<Agent> agent = std::static_pointer_cast<Agent>(Agent::createInternal(id, agentSpec.getType()));
		agent->setShape(agentSpec.getShape());
		for (SiteSpecification siteSpec : agentSpec.getSiteSpecifications()) {
			shared_ptr<Site> site = std::static_pointer_cast<Site>(Site::createInternal(siteSpec.getId(), siteSpec.getType()));
			site->setOwner(agent);
			site->setPosition(siteSpec.getPosition());
			agent->addSite(site);
		}
		return agent;
	}

	template class SimulationObjectFactory<Site, SimulationObject>;
	template class SimulationObjectFactory<Agent, SimulationObject>;
	template class SimulationObjectFactory<AgentCluster, SimulationObject>;
}

