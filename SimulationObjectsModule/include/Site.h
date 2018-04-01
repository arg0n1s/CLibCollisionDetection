#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"

namespace simobj {

	/**
		\brief Objects of this class are used to connect agents to other agents.
		That means, agents are never directly connected to other agents, but their
		sites are connected to sites of other agents.
		Sites provide its owning agent with information on where they are on the agents hull,
		what other possible site (and agent) they are connected to or if they are connected at all.
		\note Objects of this type are instantiated on the heap only, through the simulation object factory.
	*/
	class Site :
		public SimulationObject, public SimulationObjectFactory<Site, SimulationObject>
	{
	public:

		/**
			\brief Destructor
		*/
		~Site();

		/**
			\brief Call to instantiate a new Site-object.
			\returns Returns a smart pointer to the new object on the heap.
			\note This is the only way to "make" a Site-object, i.e. Constructor is private!
			\param[in] id Locally (within an agent) unique identifier of this object (used to reference sites in the simulation)
			\param[in] type Defines the type of this site (used to generate sites from a template)
		*/
		static SimObjPtr New(const unsigned long& id, const string& type);

		/**
			\brief Packs the site's internal information into a formatted readable string.
		*/
		virtual string toString() const;

		/**
		\brief Get the local position of this site.
		\returns The sites local position as a 3x1 vector in real space.
		*/
		virtual const Vector3d& getPosition() const;

		/**
		\brief Get the local orientation of this site.
		\returns The sites local orientation as quaternion.
		*/
		virtual const Quaternion& getOrientation() const;

		/**
			\brief Get the karthesian coordinates of this site object in global or local coordinates.
			\throw Exception if an unknown reference frame enumerator is used.
			\note If the site does not belong to an agent, global coordinates equal local coordinates.
			\param[in] frame reference frame enumerator (i.e.: Return global or local coordinates?)
		*/
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;

		/**
			\brief Get the orientation of this site object as a quaternion, in global or local coordinates.
			\throw Exception if an unknown reference frame enumerator is used.
			\note If the site does not belong to an agent, global orientation equals local orientation.
			\param[in] frame reference frame enumerator (i.e.: Return global or local orientation?)
		*/
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;

		/**
			\brief Set the owning Agent-object of this site.
			\param[in] owner Smart pointer to an Agent-object
		*/
		void setOwner(SimObjPtr owner);

		/**
			\brief Get the owning Agent-object of this site.
			\throw Exception if this site has no owning agent.
			\returns Smart pointer to the Agent-object owning this site.
		*/
		SimObjPtr getOwner();

		/**
			\brief Check if this site is connected to another site.
			\returns True if this site is connected to another site.
		*/
		bool isConnected() const;

		/**
			\brief Connect this site to another site (unidirectional).
			\note For a bidirectional connection, this method has to be called on the other site as well.
			\param[in] otherSite Smart pointer to a Site-object to be conected to this site.
		*/
		void connect(SimObjPtr otherSite);

	protected:

		/** \brief Constructor
			\param[in] id Locally unique (within an agent) identifier of this object (used to reference sites in the simulation)
			\param[in] type Defines the type of this site (used to generate sites from a template)
		*/
		Site(const unsigned long& id, const string& type);

		/* Flag that indicates an existing connection to another site.*/
		bool connected;

		/* Flag that indicates an existing ownership of this site by an agent.*/
		bool hasOwner;

		/* Smart pointer to the owning Agent-object.*/
		SimObjWeakPtr ownerAgent;

		/* Smart pointer to the connected site.*/
		SimObjWeakPtr otherSite;
	};
}

