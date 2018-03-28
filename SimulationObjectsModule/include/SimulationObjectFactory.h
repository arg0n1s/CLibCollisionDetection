#pragma once
#include <memory>
#include <string>
namespace simobj {

	// Forward declarations to reduce the number of includes.
	namespace specs {
		class AgentSpecification;
		class SiteSpecification;
	}

	// Aliases for used namespaces
	using std::string;
	using specs::AgentSpecification;
	using specs::SiteSpecification;
	using std::shared_ptr;

	/**
		\brief Factory class to produce instances of simulation objects, i.e. Agents-objects
		and AgentCluster-objects, Site-objects are produced implicitly.
		Resulting objects are placed on the heap, hence the factory returns smart pointers only.
	*/
	template <typename TClass, typename TInterface>
	class SimulationObjectFactory {
	public:

		/**
			\brief Generic factory function to produce a new object with super-type SimulationObject.
			\param[in] id Globally unique identifier of this object (used for reference in the simulation)
			\example shared_ptr<agent> agent = SimulationObjectFactory::create<Agent>(0,"default-type");
			\returns Smart pointer to the objects location on the heap.
		*/
		static shared_ptr<TInterface> create(const unsigned long& id, const string& type);

		/**
			\brief Factory function to produce a new Agent-object and its attached sites from a given agent specification.
			\param[in] id Globally unique identifier of this object (used to reference agent in the simulation)
			\param[in] agentSpec The agent specification defines the shape, number and location of sites and type of the agent.
			\throws Exception if a given site coordinate type inside the specs is unknown. 
			\returns Smart pointer to the objects location on the heap.
		*/
		static shared_ptr<TInterface> create(const unsigned long& id, const AgentSpecification& agentSpec);
		
	};
}