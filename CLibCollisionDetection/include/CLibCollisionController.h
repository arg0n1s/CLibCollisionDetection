#pragma once

#include <SimulationContainer.h>
#include <MetaSpecification.h>
#include <CollisionDetection.h>
#include <string>
#include <vector>
#include <VTKVisualization.h>

// Macro needed to provide function signatures in .dll files.
#ifdef CLIBCOLLISIONDETECTION_EXPORTS
#define CLIB_COLLISION_DETECTION_API __declspec(dllexport) 
#else
#define CLIB_COLLISION_DETECTION_API __declspec(dllimport) 
#endif

// Forward declarations to remove unnecessary includes.
namespace simobj {
	namespace shapes {
		class Shape;

		namespace types {
			enum ShapeType;
		}
		typedef types::ShapeType ShapeType;
	}
}

namespace clib
{
	// Aliases for used namespaces
	using vis::VTKVisualization;
	using simobj::SimulationContainer;
	using simobj::SimObjPtr;
	using simobj::specs::MetaSpecification;
	using simobj::specs::AgentSpecification;
	using simobj::specs::SiteSpecification;
	using simobj::specs::CoordinateType;
	using SiteSpecArray = std::vector<SiteSpecification>;
	using AgentSpecArray = std::vector<AgentSpecification>;
	using std::string;
	using ShapePtr = std::shared_ptr<simobj::shapes::Shape>;
	using simobj::shapes::ShapeType;
	using collision::CollisionDetection;

	/**
		\brief Helper function to create a site specification with a locally unique (i.e. within an agent) id,
		at parametrized or unparametrized coordinates depending on the given coordinate type enumerator.
		\note For more information on coordinate types see simobj::specs::CoordinateType. Coordinate components depend
		heavly on the shape of the intended agents this site should be attached to
		for more information on coordinate components see simobj::shapes::Shape.
		\param[in] id Locally unique (i.e. within an agent) id of the site.
		\param[in] c1 First coordinate component of the site on the hull of a possible owning agent.
		\param[in] c2 Second coordinate component of the site on the hull of a possible owning agent.
		\param[in] c3 Third coordinate component of the site on the hull of a possible owning agent.
		\param[in] cType Coordinate type enumerator used to correctly interpret the location coordinates.
		\returns New SiteSpecification-object created according to the given parameters.
	*/
	CLIB_COLLISION_DETECTION_API SiteSpecification createSiteSpecification(const unsigned long& id, const double& c1, const double& c2, const double& c3, const CoordinateType& cType);

	/**
		\brief Helper function to create a new agent specification template, from which new agents of this type
		may be instantiated during or at the beginning of the simulation.
		\param[in] type Unique identifier of this agent type, used by the factory to produce Agent-objects with the specified properties.
		\param[in] shape Smart pointer to the shape object defining this agent types geometric properties.
		\param[in] siteSpecs Array of siteSpecs defining the sites an agent of this type should have.
		\returns New AgentSpecification-object created according to the given parameters.
	*/
	CLIB_COLLISION_DETECTION_API AgentSpecification createAgentSpecification(const string& type, ShapePtr shape, SiteSpecArray siteSpecs);

	/**
		\brief Helper function to create a new MetaSpecification-object which contains the given collection
		of AgentSpecification-objects and SiteSpecification-objects used to produce new agents of a given type
		during or at the beginning of the simulation.
		\param[in] siteSpecs Array of agentSpecs defining the agents of a particular type.
		\returns New MetaSpecification-object created according to the given parameters.
	*/
	CLIB_COLLISION_DETECTION_API MetaSpecification createMetaSpecification(AgentSpecArray agentSpecs);

	/**
		\brief Main class for the CLibCollisionDetection library. Objects of this class provide access to functionality
		that may be used for collision detection of molecules in pattern based simulation of biochemical reactions. 
		This library can also be used for other purposes where collision checking might be useful, since its representation of objects
		is quite generic (spheres, cylinders, other parametrized bodies).
		Main functionionalities are: 
		1. Creation of new objects (aka. agents) from templates (aka. MetaSpecifications) used to check for collision.
		2. Grouping of agens that are connected to each other through fixed connections.
		3. Groups of agents are clusters, that apply rotation and translation to all contained agents.
		4. Fixed connections are expressed through sites attached to an agent, that are linked to other sites attached to another agent.
		5. Collision checking of agents with clusters or clusters with clusters.
		6. Efficient collision checking though the use of OctTrees.
		7. Visualization of Agents, Sites, Clusters and OctTrees with VTK.
	*/
	class CLibCollisionController
	{
	public:
		
		/**
			\brief Construct a new collision controller object. 
			Starts error logging, initializes visualization and sets collision detector 
			minimal cell diameter to 2.0, aswell as initial tree diameter to 4.0.
		*/
		CLIB_COLLISION_DETECTION_API CLibCollisionController(const double& initialTreeDiameter = 4.0, const double& minimalCellDiameter = 2.0, const bool rescalingOn = true);

		/**
			\brief Construct a new collision controller object with meta specifications.
			Starts error logging, initializes visualization and sets collision detector
			minimal cell diameter to 2.0, aswell as initial tree diameter to 4.0.
			\param[in] metaSpecs Templates from which agents can be produced for the simulation container.
		*/
		CLIB_COLLISION_DETECTION_API CLibCollisionController(const MetaSpecification& metaSpecs, const double& initialTreeDiameter = 4.0, const double& minimalCellDiameter = 2.0, const bool rescalingOn = true);

		/**
			\brief Destructor, ends logging on shut-down.
		*/
		CLIB_COLLISION_DETECTION_API ~CLibCollisionController();

		/**
		\brief Helper function to create a shape object with a given shape type and the corresponding parameters.
		\note For more information on coordinate components and available shape types see simobj::shapes::Shape.
		\param[in] shapeType Shape type enumerator defining the required shape.
		\param[in] Args... Any number of parameters of the type double.
		\returns Smart pointer to the new shape created according to the given parameters.
		*/
		template<typename... Args>
		static CLIB_COLLISION_DETECTION_API ShapePtr createShape(const ShapeType& shapeType, Args... args);

		/**
			\brief Create a new empty agent cluster with the given unique id and some type.
			\param[in] id Unique id of the new agent cluster.
			\param[in] type Template type of the agent cluster (currently unused, user may set any value).
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool createAgentCluster(const unsigned long& id, const string& type);

		/**
			\brief Create a new agent with the given unique id and some type.
			\param[in] id Unique id of the agent.
			\param[in] type Name of the agent specification template from which the agent should be created.
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool createAgent(const unsigned long& id, const string& type);

		/**
			\brief Get the CollisionDetection-object used in this controller. 
			\return Reference to the contained collision detector.
		*/
		CLIB_COLLISION_DETECTION_API CollisionDetection& getCollisionDetector();

		/**
			\brief Set the minimal leaf diameter of internally constructed OctTrees, i.e.
			the minimal symmetric allowed width/heigth/length of a leaf-node (-> resolution)
			\param[in] minimalCellDiameter  Minimal leaf diameter of internally constructed OctTrees
			\note This paramter greatly influences how long it takes to construct a tree and how precise
			a list of collision candidates is.
		*/
		CLIB_COLLISION_DETECTION_API void setMinimalLeafDiameter(const double& leafDiameter);

		/**
			\brief Set the initial tree diameter of internally constructed OctTrees, i.e.
			the initial symmetric allowed width/heigth/length of new tree's roots.
			\param[in] initialTreeDiameter Initial tree diameter of internally constructed OctTrees
			\note This paramter greatly influences how long it takes to construct a tree and how often a
			tree has to be resized initially.
		*/
		CLIB_COLLISION_DETECTION_API void setInitialRootDiameter(const double& rootDiameter);

		/**
			\brief Returns the Agent-object mapped to the given id.
			\throw Exception when the agent mapped to the given id is not present within this controller.
			\param[in] id Unique identifier of an agent.
		*/
		CLIB_COLLISION_DETECTION_API SimObjPtr getAgent(const unsigned long& id);

		/**
			\brief Returns the AgentCluster-object mapped to the given id.
			\throw Exception when the agent cluster mapped to the given id is not present within this controller.
			\param[in] id Unique identifier of an agent cluster.
		*/
		CLIB_COLLISION_DETECTION_API SimObjPtr getAgentCluster(const unsigned long& id);

		/**
			\brief Add an existing agent to an existing cluster.
			\param[in] agentId Unique identifier of the agent.
			\param[in] clusterId Unique identifier of the cluster.
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool addAgentToCluster(const unsigned long& agentId, const unsigned long& clusterId);

		/**
			\brief Add a single agent with the given id to the collision detector.
			I.e Add the agent to previously constructed OctTree, if no tree is present a new one will be constructed.
			\param[in] agentId Unique identifier of the agent.
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool addAgentToCollisionDetector(const unsigned long& agentId);

		/**
			\brief Add a single agent with the given id belonging to the given cluster to the collision detector.
			I.e Add the agent to previously constructed OctTree, if no tree is present a new one will be constructed.
			\param[in] agentId Unique identifier of the agent.
			\param[in] clusterId Unique identifier of the cluster the given agent belongs to.
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool addAgentToCollisionDetector(const unsigned long& agentId, const unsigned long& clusterId);

		/**
			\brief Add an agent cluster with the given id to the collision detector.
			I.e Construct a new Oct-Tree from the agent cluster with the given id.
			\return True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool addAgentClusterToCollisionDetector(const unsigned long& clusterId);

		/**
			\brief Find the nearest agent belonging to the given cluster with respect to the given agent.
			\param[in] agentId Unique identifier of the query-agent.
			\param[in] clusterId Unique identifier of the cluster being queried.
			\param[out] nearest Smart pointer to the Agent-object nearest to the query-agent.
			\return True if no error was logged and a nearest agent exists.
		*/
		CLIB_COLLISION_DETECTION_API const bool findNearestToAgent(const unsigned long& agentId, const unsigned long& clusterId, SimObjPtr nearest);

		/**
			\brief Check for a collision between two agents.
			\param[in] agt1 Unique identifier of the first agent.
			\param[in] agt2 Unique identifier of the second agent.
			\return True if a collision occurs.
		*/
		CLIB_COLLISION_DETECTION_API bool checkCollisionBetweenAgents(const unsigned long& agt1, const unsigned long& agt2);

		/**
			\brief Calculate the distance between the hulls of two agents.
			\param[in] agt1 Unique identifier of the first agent.
			\param[in] agt2 Unique identifier of the second agent.
			\return Hull-to-hull distance of the fiven agents. Negative distance indicates a collision.
		*/
		CLIB_COLLISION_DETECTION_API double distanceBetweenAgents(const unsigned long& agt1, const unsigned long& agt2);

		/**
			\brief Demonstration method that connects two agents at the given sites with each other 
			and automatically builds a new AgentCluster, if none of the given agents belong to one. 
			Doesn't check for collisions.
			\note This will fail, if both agents belong to an agent cluster or if the second agent belongs to an agent cluster.
			\param[in] agt1 Unique identifier of the first agent.
			\param[in] agt2 Unique identifier of the second agent.
			\param[in] st1 Unique identifier of the first agent site.
			\param[in] st2 Unique identifier of the second agent site.
			\throws Exception if any of the given IDs do not exist.
		*/
		CLIB_COLLISION_DETECTION_API void connectAgents(const unsigned long& agt1, const unsigned long& agt2, const unsigned long& st1, const unsigned long& st2);

		/**
			\brief Displays an agent with the given ID using VTK.
			\param[in] id Unique identifier of an agent.
			\returns True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool displayAgent(const unsigned long& id);

		/**
			\brief Displays an agent cluster with the given ID using VTK.
			\param[in] id Unique identifier of an agent cluster.
			\returns True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool displayAgentCluster(const unsigned long& id);

		/**
			\brief Displays an oct tree and its contained agent cluster with the given ID using VTK.
			\param[in] id Unique identifier of an agent cluster.
			\returns True if no error was logged.
		*/
		CLIB_COLLISION_DETECTION_API bool displayClusterCollisionTree(const unsigned long& clusterId);

		/**
			\brief Packs the controllers's internal information into a formatted readable string.
		*/
		CLIB_COLLISION_DETECTION_API string toString();

	private:

		/* Collection of meta specifications to produce new agents from. */
		MetaSpecification metaSpecs;

		/* Object containing and managing agents, agent clusters and sites. */
		SimulationContainer simContainer;

		/* Object containing OctTrees and managing collision detection. */
		CollisionDetection collisionDetector;

		/* Object containing VTK render objects and logic. */
		VTKVisualization vtkVis;

		/* Used for tracking last created agent cluster id. */
		unsigned long nextClusterID;

		/**
			\brief Generates the next unused agent cluster id.
		*/
		CLIB_COLLISION_DETECTION_API const unsigned long generateNextClusterID();
	};
}