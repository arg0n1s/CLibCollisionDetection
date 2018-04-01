#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"
#include <unordered_map>

namespace simobj {

	namespace shapes {
		class Shape;
	}

	// Aliases for used namespaces
	using std::string;
	using shapes::Shape;
	using ShapePtr = std::shared_ptr<Shape>;
	using SimObjPtr = SimulationObject::SimObjPtr;
	using SimObjWeakPtr = SimulationObject::SimObjWeakPtr;
	using SitesMap = std::unordered_map<unsigned long, SimObjPtr>;

	/** 
		\brief Objects of this class are the main protagonists used in the simulation.
		Agents contain their location and orientation in local coordinates, but may transform
		to global coordinates when necessary, given a global context (i.e. they belong to an Agent Cluster).
		Agents may be connected to other agents through attached sites, which are placed on the hull of the 
		geometric object representing the 3D properties of the respective agent object.
		3D properties of an agent object are represented through a shape object, which may be of type Sphere, 
		Cylinder, Ellipsoid, etc.
		The shape of an Agent, its orientation and position are the main components used to determine collisions 
		between agents in the simulation.
		\note Objects of this type are instantiated on the heap only, through the simulation object factory.
	*/
	class Agent :
		public SimulationObject, public SimulationObjectFactory<Agent, SimulationObject>
	{
	public:

		/** 
			\brief Destructor
		*/
		~Agent();

		/**
			\brief Call to instantiate a new Agent-object.
			\returns Returns a smart pointer to the new object on the heap.
			\note This is the only way to "make" an Agent-object, i.e. Constructor is private!
			\param[in] id Globally unique identifier of this object (used to reference agent in the simulation)
			\param[in] type Defines the type of this agent (used to generate agents from a template)
		*/
		static SimObjPtr New(const unsigned long& id, const string& type);

		/** 
			\brief Packs the agent's internal information into a formatted readable string.
		*/
		virtual string toString() const;

		/**
		\brief Get the local position of this agent.
		\returns The agents local position as a 3x1 vector in real space.
		*/
		virtual const Vector3d& getPosition() const;

		/**
		\brief Get the local orientation of this agent.
		\returns The agents local orientation as quaternion.
		*/
		virtual const Quaternion& getOrientation() const;

		/** 
			\brief Get the karthesian coordinates of this agent object in global or local coordinates.
			\throw Exception if an unknown reference frame enumerator is used.
			\note If the agent does not belong to a Cluster, global coordinates equal local coordinates.
			\param[in] frame reference frame enumerator (i.e.: Return global or local coordinates?)
		*/
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;

		/** 
			\brief Get the orientation of this agent object as a quaternion, in global or local coordinates.
			\throw Exception if an unknown reference frame enumerator is used.
			\note If the agent does not belong to a Cluster, global orientation equals local orientation.
			\param[in] frame reference frame enumerator (i.e.: Return global or local orientation?)
		*/
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;

		/** 
			\brief Adds a Site-object to this agent.
			\throw Exception if site with identical id is already present.
			\note Sites are used to connect agents with each other.
			\param[in] site Smart pointer to a Site-object
		*/
		void addSite(SimObjPtr site);

		/** 
			\brief Set the AgentCluster-object, this agent should belong to.
			\note Agent clusters are used to move/rotate multiple agents at once
			and provides a global frame of reference for coordinate transformations.
			\param[in] cluster Smart pointer to an AgentCluster-object
		*/
		void setAgentCluster(SimObjPtr cluster);

		/** 
			\brief Set the Shape-object of this agent.
			\note Shapes define geometric 3D properties of an agent and are used to check for collisions.
			\param[in] shape Smart pointer to a Shape-object
		*/
		void setShape(ShapePtr shape);

		/** 
			\brief Returns a Site-object of this agent with a certain id.
			\throw Exception if site with given id does not exist.
			\note Sites are used to connect agents with each other.
			\param[in] id Locally unique id of a Site-object
		*/
		SimObjPtr getSite(const unsigned long& id);

		/** 
			\brief Returns an unordered hash map which contains every site that belongs to this agent.
			\note Key: Site id (unique within this agent), Value: Smart pointer to a Site-object
		*/
		const SitesMap& getAllSites() const;

		/** 
			\brief Returns the Shape-object of this agent.
			\throw Exception if this agent does not have a shape object assigned to it.
			\note Shapes define geometric 3D properties of an agent and are used to check for collisions.
		*/
		ShapePtr getShape();

		/** 
			\brief Returns the AgentCluster-object, this agent belongs to.
			\throw Exception when this agent doesn't belong to any cluster.
			\note Agent clusters are used to move/rotate multiple agents at once
			and provides a global frame of reference for coordinate transformations.
		*/
		SimObjPtr getAgentCluster();

		/**
			\brief Check to see if a site with the given id is part of this agent.
			\param[in] id Id of the site in question.
			\returns True if site belongs to agent.
		*/
		bool isSiteAtAgent(const unsigned long& id) const;

		/**
			\brief Check to see if this agent is part of any cluster.
			\returns True if this cluster belongs to an agent cluster.
		*/
		bool isInAnyCluster() const;

		/**
			\brief Check to see if this agent belongs to the given cluster.
			\returns True if this cluster belongs to the given agent cluster.
		*/
		bool isAgentCluster(SimObjPtr cluster) const;
	protected:

		/** \brief Constructor
			\param[in] id Unique identifier of this object (used to reference agent in the simulation)
			\param[in] type Defines the type of this agent (used to generate agents from a template)
		*/
		Agent(const unsigned long& id, const string& type);

		/*Map of sites attached to this agent.*/
		SitesMap sites;

		/*Shape object of this agent.*/
		ShapePtr shape;

		/*Weak pointer to the cluster this agent belongs to.*/
		SimObjWeakPtr cluster;

		/*Flag to signal whether this agents belongs to a cluster.*/
		bool belongsToCluster;

		/*Flag to signal whether this agents has a shape object assigned to it.*/
		bool hasShape;
	};
}