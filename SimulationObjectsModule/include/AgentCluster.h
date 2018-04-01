#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"
#include <unordered_map>

namespace simobj {

	// Aliases for used namespaces
	using SimObjPtr = SimulationObject::SimObjPtr;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;

	/**
		\brief Objects of this class are used as containers for Agent-objects, to group
		connected agents and simplify rotation and translation of multiple objects at the same time.
		Hence agent clusters provide agents with a global frame of reference to transfrom
		agent coordinates to global, from local coordinates.
		If an agent cluster is moved, only its global reference frame is moved, local coordinates of
		possible contained agents are left untouched. When a global representation of agent
		coordinates is required, local coordinates are transformed ad-hoc trough the use of the 
		rotation and translation of the agent cluster.
		\note Objects of this type are instantiated on the heap only, through the simulation object factory.
	*/
	class AgentCluster :
		public SimulationObject, public SimulationObjectFactory<AgentCluster, SimulationObject>
	{
	public:

		/** 
			\brief Destructor
		*/
		~AgentCluster();

		/**
			\brief Call to instantiate a new AgentCluster-object.
			\returns Returns a smart pointer to the new object on the heap.
			\note This is the only way to "make" an AgentCluster-object, i.e. Constructor is private!
			Cluster types are currently unused, user may set any type.
			\param[in] id Globally unique identifier of this object (used to reference agent cluster in the simulation)
			\param[in] type Defines the type of this agent cluster (used to generate agent clusters from a template)
		*/
		static SimObjPtr New(const unsigned long& id, const string& type);

		/**
			\brief Packs the agent cluster's internal information into a formatted readable string.
		*/
		virtual string toString() const;

		/**
		\brief Get the local position of this agent cluster.
		\returns The agent clusters local position as a 3x1 vector in real space.
		*/
		virtual const Vector3d& getPosition() const;

		/**
		\brief Get the local orientation of this agent cluster.
		\returns The agent clusters local orientation as quaternion.
		*/
		virtual const Quaternion& getOrientation() const;

		/**
			\brief Get the karthesian coordinates of this agent cluster object in global or local coordinates.
			\note An agent cluster always returns global coordinates independet of the given reference frame.
			\param[in] frame reference frame enumerator (i.e.: Return global or local coordinates?)
		*/
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;

		/**
			\brief Get the orientation of this agent cluster object as a quaternion, in global or local coordinates.
			\note An agent cluster always returns global orientation independet of the given reference frame.
			\param[in] frame reference frame enumerator (i.e.: Return global or local orientation?)
		*/
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;

		/**
			\brief Insert a new agent into this cluster.
			\throw Exception when the given agents id is already present within this cluster.
			\param[in] agent Smart pointer to the Agent-object to be inserted.
		*/
		void insertAgent(SimObjPtr agent);
		
		/**
			\brief Returns the Agent-object mapped to the given id.
			\throw Exception when the agent mapped to the given id is not present within this cluster.
			\param[in] id Unique identifier of an agent.
		*/
		SimObjPtr getAgent(const unsigned long& id);

		/**
			\brief Get the internal collection of agents within this cluster.
			\returns A unorderd hash map containing agents of this cluster, mapped to their ids.
		*/
		const unordered_map& getAllAgents() const;

		/**
			\brief Check whether an Agent-object mapped to the given id is in this cluster.
			\param[in] id Unique identifier of an agent.
			\returns True if agent mapped to the given id is present within this cluster.
		*/
		const bool isAgentInCluster(const unsigned long& id) const;

	protected:

		/** 
			\brief Constructor
			\note Cluster types are currently unused, user may set any type.
			\param[in] id Unique identifier of this object (used to reference agent cluster in the simulation)
			\param[in] type Defines the type of this agent cluster (used to generate agent clusters from a template)
		*/
		AgentCluster(const unsigned long& id, const string& type);

		/*Map of contained agents in this cluster.*/
		unordered_map agents;

	};
}