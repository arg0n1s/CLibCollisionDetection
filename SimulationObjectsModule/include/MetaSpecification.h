#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <memory>

namespace simobj {
	
	// Forward declaration to reduce the number of includes.
	namespace shapes {
		class Shape;
	}
	
	namespace specs {

		// Alias for used namespaces
		using std::string;

		namespace types {

			/**
				\brief Enumerator used to signal which method is used to specify the location
				of a site on the hull of its owning agent. 
			*/
			enum CoordinateType {
				/* Absolute karthesian coordinates of the sites location, with regard to the agents origin. */
				KarthesianAbsolute, 
				/* A 3x1 vector in karthesian coordinates that points from the agents origin to its hull.
				The intersection of this pointer with the agent's hull specifies the site's location. */
				KarthesianPointerToHull, 
				/* Absolute parametrized coordinates of the sites location, with regard to the agents origin and its shape type. */
				ParametricAbsolute, 
				/* A 3x1 vector in parametrized coordinates that points from the agents origin to its hull.
				The intersection of this pointer with the agent's hull specifies the site's location. */
				ParametricPointerToHull
			};

		}

		// Typedef for ease of use
		typedef types::CoordinateType CoordinateType;

		// Forward declarations to reduce the number of includes.
		class AgentSpecification;
		class SiteSpecification;

		// Aliases for used namespaces
		using AgentSpecsMap = std::unordered_map<string, AgentSpecification>;
		using SiteSpecsMap = std::unordered_map<unsigned long, SiteSpecification>;
		using SiteSpecsArray = std::vector<SiteSpecification>;
		using Eigen::Vector3d;
		using simobj::shapes::Shape;
		using ShapePtr = std::shared_ptr<Shape>;
		using types::CoordinateType;

		/**
			\brief Objects of this class contain information on how to create numerous
			agents of a certain type, including the agents attached sites.
			I.e. a MeteSpecification-object is no more than a collection of AgentSpecification-objects.
		*/
		class MetaSpecification {
		public:
			/**
				\brief Constructor
			*/
			MetaSpecification();

			/**
				\brief Add the given agent specification to this meta specification.
				\param[in] agentSpec AgentSpecification-object to be added.
				\throws Exception if an agent specification with the given type is already present within this agent specification.
			*/
			void addAgentSpecification(const AgentSpecification& agentSpec);

			/**
				\brief Get the agent specification with the given type.
				\param[in] type Type of the requested agent specification.
				\throws Exception if an agent specification with the given type is not present within this agent specification.
			*/
			const AgentSpecification& getAgentSpecification(const string& type) const;

			/**
				\brief Check if the agent specification with the given type is present within this agent specification.
				\param[in] type Type of the requested agent specification.
				\returns True if agent specification with the given type is present.
			*/
			const bool isAgentInSpecs(const string& type) const;

		private:

			/* An unordered map of agent specifications, with agent specification types as key and the specs as value. */
			AgentSpecsMap agentSpecs;
		};

		/**
			\brief Objects of this class contain information on how to create a particular agent
			of a certain type, including the agents attached sites.
			Agent specifications are used as templates by the factory to produce Agent-objects with the specified properties.
			Agent specifications carry the shape information of an agent type and have the collection of site specifications
			used to attach sites to an agent type.
		*/
		class AgentSpecification {
		public:

			/**
				\brief Constructs an agent specification object with the given
				type and shape.
				\param[in] type Unique identifier of this agent type, later used by the factory to produce Agent-object with the specified properties.
				\param[in] shape Smart pointer to the shape object defining this agent types geometric properties.
			*/
			AgentSpecification(const string& type, ShapePtr shape);

			/**
				\brief Get the type of this agent specification.
				\returns String denoting this agent specifications type.
			*/
			const string& getType() const;

			/**
				\brief Get the shape of this agent specification.
				\returns Smart pointer to the shape of this agent specification.
			*/
			const ShapePtr getShape() const;

			/**
				\brief Add the given site specification to this agent specification.
				\param[in] siteSpec SiteSpecification-object to be added.
				\throws Exception if a site with the given id (Not type!) is already present within this agent specification.
			*/
			void addSiteSpecification(const SiteSpecification& siteSpec);

			/**
				\brief Get all site specifications attached to this agent specification.
				\returns An unordered map of site specifications, with site specification ids as key and the specs as value.
			*/
			const SiteSpecsArray getSiteSpecifications() const;

			/**
				\brief Check if a site specification with the given id is attached to this agent specification.
				\returns True if site specification exists in this agent specification.
			*/
			const bool isSiteInSpecs(const unsigned int& id) const;
		private:

			/* Type of this agent specification. */
			string type;

			/* An unordered map of site specifications, with site specification ids as key and the specs as value. */
			SiteSpecsMap siteSpecs;

			/* Smart pointer to the shape of this agent specification. */
			ShapePtr shape;
		};

		/**
			\brief Objects of this class contain information on where sites are located on their owning agents hull.
			Site specifications are used by the factory to create and attach the specified sites to their owning agents.
		*/
		class SiteSpecification {
		public:

			/**
				\brief Constructs a site specification object with the given id, type, location and coordinate type enumerator.
				\param[in] id Locally unique identifier of a site of this type within its owning agent.
				\param[in] type Unique identifier of this site type, later used by the factory to produce Site-objects with the specified properties.
				\param[in] coordinates Location of a site of this type on the hull of its owning agent. 
				\param[in] cType Coordinate type enumerator used to correctly interpret the location coordinates.
			*/
			SiteSpecification(const unsigned long& id, const string& type, const Vector3d& coordinates, const CoordinateType& cType);

			/**
				\brief Get the locally unique identifier of a site of this type within its owning agent.
				\returns Id of a site of this type.
			*/
			const unsigned long& getId() const;

			/**
				\brief Get the unique identifier of this site type, later used by the factory to 
				produce Site-objects with the specified properties.
				\returns Type of this site specification.
			*/
			const string& getType() const;

			/**
				\brief Get the location of a site of this type on the hull of its owning agent. 
				\returns Coordinates on the owning agents hull.
			*/
			const Vector3d& getCoordinates() const;

			/**
				\brief Get the coordinate type enumerator used to correctly interpret the location coordinates.
				\returns Coordinate type enumerator of this site specification.
			*/
			const CoordinateType& getCoordinateType() const;
		private:

			/* Locally unique identifier of a site of this type within its owning agent. */
			unsigned long id;

			/* Unique identifier of this site type, later used by the factory to 
				produce Site-objects with the specified properties. */
			string type;

			/* Location of a site of this type on the hull of its owning agent. */
			Vector3d coordinates;

			/* Coordinate type enumerator used to correctly interpret the location coordinates. */
			CoordinateType cType;
		};
	}
}