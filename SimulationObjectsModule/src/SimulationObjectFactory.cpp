#include "../include/SimulationObjectFactory.h"
#include "../include/MetaSpecification.h"
#include "../include/SimulationObject.h"
#include "../include/Agent.h"
#include "../include/Site.h"
#include "../include/AgentCluster.h"
#include "../include/Shape.h"

namespace simobj {

	using specs::CoordinateType;

	template <typename TClass, typename TInterface>
	shared_ptr<TInterface> SimulationObjectFactory<TClass, TInterface>::create(const unsigned long& id, const string& type) { 
		return TClass::createInternal(id, type); 
	}
	template <typename TClass, typename TInterface>
	shared_ptr<TInterface> SimulationObjectFactory<TClass, TInterface>::create(const unsigned long& id, const AgentSpecification& agentSpec) {
		ShapePtr shape = agentSpec.getShape();
		shared_ptr<Agent> agent = std::static_pointer_cast<Agent>(Agent::createInternal(id, agentSpec.getType()));
		agent->setShape(shape);

		for (SiteSpecification siteSpec : agentSpec.getSiteSpecifications()) {
			shared_ptr<Site> site = std::static_pointer_cast<Site>(Site::createInternal(siteSpec.getId(), siteSpec.getType()));
			site->setOwner(agent);
			switch (siteSpec.getCoordinateType()) {
				case CoordinateType::KarthesianAbsolute : {
					site->setPosition(siteSpec.getCoordinates());
					break;
				}
				case CoordinateType::KarthesianPointerToHull : {
					Vector3d position = shape->hullIntersectionFromKarthPointer(siteSpec.getCoordinates());
					site->setPosition(position);
					break;
				}
				case CoordinateType::ParametricPointerToHull: {
					Vector3d position = shape->hullIntersectionFromParametrizedPointer(siteSpec.getCoordinates());
					site->setPosition(position);
					break;
				}
				case CoordinateType::ParametricAbsolute: {
					Vector3d position = shape->parametrizedToKarthesianCoordinates(siteSpec.getCoordinates());
					site->setPosition(position);
					break;
				}
				default: {
					throw std::runtime_error("Given coordinate type does not exist or is not supported!");
				}
			}
			agent->addSite(site);
		}
		return agent;
	}

	template class SimulationObjectFactory<Site, SimulationObject>;
	template class SimulationObjectFactory<Agent, SimulationObject>;
	template class SimulationObjectFactory<AgentCluster, SimulationObject>;
}

