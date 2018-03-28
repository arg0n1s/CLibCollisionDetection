#pragma once

#include <unordered_map>
#include <memory>
#include "../include/MetaSpecification.h"

namespace simobj {

	// Forward declaration to reduce the number of includes.
	class SimulationObject;

	// Aliases for used namespaces
	using SimObjPtr = std::shared_ptr<SimulationObject>;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;
	using specs::MetaSpecification;

	/**
		\brief A Simulation container contains all simulation objects and offers
		functionality to add new objects to and retrieve objects from the container.
		It is also possible to construct and add new objects from pre-defined templates,
		aka Meta-Specifications.
	*/
	class SimulationContainer
	{
	public:

		/**
			\brief Default-Constructor
		*/
		SimulationContainer();

		/**
			\brief Construct a simulation container with pre-defined templates, aka. Meta-Specifications.
			\param[in] metaSpecs Meta-Specifications to construct Agent-objects from.
		*/
		SimulationContainer(const MetaSpecification& metaSpecs);

		/**
			\brief Construct and add a new agent with a specified template type and a given unique id.
			\param[in] id Unique identifier of a new agent (used to reference agent in the simulation)
			\param[in] type Defines the type of a new agent (used to generate agents from a template)
			\throws Exception if given id is already present with this container.
		*/
		void addAgent(const unsigned long& id, const string& type);

		/**
			\brief Add an existing agent to an existing cluster.
			\param[in] agentId Unique identifier of the agent.
			\param[in] clusterId Unique identifier of the cluster.
			\throws Exception if either of the given id is not present with this container.
			\throws Exception if given agent id is already present with the to be assgined cluster.
		*/
		void addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId);

		/**
			\brief Construct and add a new cluster with a specified template type and a given unique id.
			\note Cluster types are currently unused, user may set any type.
			\param[in] id Unique identifier of a new cluster (used to reference cluster in the simulation)
			\param[in] type Defines the type of a new cluster (used to generate cluster from a template)
			\throws Exception if given id is already present with this container.
		*/
		void addAgentCluster(const unsigned long& id, const string& type);

		/**
			\brief Connect two sites from different agents with each other.
			\note Agents that are connected through sites are not automatically added into the same cluster.
			\param[in] id Unique identifier of the first site
			\param[in] id Unique identifier of the second site
		*/
		void connectSites(SimObjPtr site1, SimObjPtr site2);

		/**
			\brief Returns the Agent-object mapped to the given id.
			\throw Exception when the agent mapped to the given id is not present within this container.
			\param[in] id Unique identifier of an agent.
		*/
		SimObjPtr getAgent(const unsigned long& id);

		/**
			\brief Returns the AgentCluster-object mapped to the given id.
			\throw Exception when the agent cluster mapped to the given id is not present within this container.
			\param[in] id Unique identifier of an agent cluster.
		*/
		SimObjPtr getAgentCluster(const unsigned long& id);

		/**
			\brief Check whether an Agent-object mapped to the given id is in this container.
			\param[in] id Unique identifier of an agent.
			\returns True if agent mapped to the given id is present within this container.
		*/
		const bool isAgentInContainer(const unsigned long& id) const;

		/**
			\brief Check whether an AgentCluster-object mapped to the given id is in this container.
			\param[in] id Unique identifier of an agent cluster.
			\returns True if agent cluster mapped to the given id is present within this container.
		*/
		const bool isAgentClusterInContainer(const unsigned long& id) const;

		/**
			\brief Packs the container's internal information into a formatted readable string.
		*/
		string toString() const;
	private:

		/*Map of contained agents in this container.*/
		unordered_map agents;
		
		/*Map of contained agent clusters in this container.*/
		unordered_map clusters;

		/*Collection of templates to create new agents from.*/
		MetaSpecification metaSpecs;
	};
}
