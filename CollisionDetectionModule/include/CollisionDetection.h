#pragma once
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace simobj {
	// Forward declaration to remove unnecessary includes.
	class SimulationObject;
	// Alias for used namespace
	using SimObjPtr = std::shared_ptr<SimulationObject>;
}

namespace collision {
	namespace octtree {
		// Forward declaration to remove unnecessary includes.
		template <typename T>
		class OctTree;

	}

	// Aliases for used namespaces
	using std::shared_ptr;
	using octtree::OctTree;
	using TreePtr = shared_ptr<OctTree<unsigned int>>;
	using TreeMap = std::unordered_map<unsigned int, TreePtr>;
	using simobj::SimulationObject;
	using simobj::SimObjPtr;
	using IDSet = std::unordered_set<unsigned int>;

	/**
		\brief Objects of this class are used to provide collision checking functionality
		for Agent-objects used in the simulation. To check for collisions between agents,
		one of those agent must be part of an agent cluster. A cluster is required 
		to generate an OctTree, with wich nearest neighbor search is performed to find 
		collision candidates. Collision candidates are then checked individually for
		possible collisions.
		\note At the moment only spheric or cylindric objects are supported, more to come in the future (To Do!).
		Also OctTrees are constructed from the inserted object's bounding box, which
		at the moment does not account for orientation of the object (To Do!).
		Additionally queries for collision candidates are done with query points (i.e. zero spatial expansion)
		In the future queries should be done with oriented query-boxes (To Do!).
	*/
	class CollisionDetection {
	public:

		/**
			\brief Constructs a CollisionDetection-object with default values for inital tree width (16.0),
			minimal leaf size (2.0) and allowRescaling flag (false).
		*/
		CollisionDetection(const double& initialTreeDiameter = 16.0, const double& minimalCellDiameter = 2.0, const bool rescalingOn = false);

		/**
			\brief Set the initial tree diameter of internally constructed OctTrees, i.e.
			the initial symmetric allowed width/heigth/length of new tree's roots.
			\param[in] initialTreeDiameter Initial tree diameter of internally constructed OctTrees
			\note This paramter greatly influences how long it takes to construct a tree and how often a
			tree has to be resized initially.
		*/
		void setInitialTreeDiameter(const double& initialTreeDiameter);

		/**
			\brief Set the minimal leaf diameter of internally constructed OctTrees, i.e.
			 the minimal symmetric allowed width/heigth/length of a leaf-node (-> resolution)
			\param[in] minimalCellDiameter  Minimal leaf diameter of internally constructed OctTrees
			\note This paramter greatly influences how long it takes to construct a tree and how precise
			a list of collision candidates is.
		*/
		void setMinimalCellDiameter(const double& minimalCellDiameter);

		/**
			\brief Set the resize allowed flag for new trees.
			\param[in] resizeOn If true, trees are allowed to expand themselves to accomodate 
			for new objects outside of their inital root sub-space.
		*/
		void setAllowRescaling(const bool rescalingOn);

		/**
			\brief Check if a cluster with the given ID is registered in this collision detector,
			i.e. has an OctTree constructed from it.
			\param[in] id Unique identifier of an agent cluster.
			\returns True if cluster is in this collision detector.
		*/
		const bool isClusterInTree(const unsigned int& id) const;

		/**
			\brief Construct an OctTree from the given cluster, where its agents are inserted into
			the new tree's nodes as unqiue ids.
			\param[in] cluster Smart pointer to the inserted cluster object.
		*/
		void makeTreeFromCluster(SimObjPtr cluster);

		/**
			\brief Add a single agent to a previously constructed OctTree.
			\param[in] agent Smart pointer to the inserted agent object.
		*/
		void addAgentToTree(SimObjPtr agent);

		/**
			\brief Get the OctTree with the given unique cluster id.
			\param[in] id Unique cluster id, shared by trees constructed from a cluster.
			\returns Smart pointer to an OctTree-object.
		*/
		TreePtr getTree(const unsigned int& id);

		/**
			\brief Check for a collision between agents of a cluster and a candidate agent.
			This method may also return the closest collision candidate its nearest distance,
			even if no collision occurs.
			\param[in] cluster Smart pointer to an Agent-Cluster object to be checked.
			\param[in] ignoreIDs Agent ids that should be ignored during collsion checking.
			\param[in] candidate Agent that should be checked for collision with given Agent-Cluster.
			\param[in] nearest Agent of the cluster that is nearest to the given candidate.
			\param[in] nearestDistance Distance to the agent nearest to the given candidate.
			\returns True if collision occured.
		*/
		const bool checkForCollision(SimObjPtr cluster, const IDSet& ignoreIDs, SimObjPtr candidate, SimObjPtr& nearest, double& nearestDistance);

		/**
			\brief Calculate the closest inter-hull distance of two agents with a particular shape.
			\note At the moment only spheres and cylinders are supported, more to come.. (ToDo!)
			\param[in] body1 First body to be used for distance calculation.
			\param[in] body2 Second body to be used for distance calculation.
			\returns Closest distance between the two body's hulls. Negative values indicate collision.
		*/
		const double calcBodyToBodyDistance(SimObjPtr body1, SimObjPtr body2) const;
	private:

		/* Hash map of stored OctTrees belonging to agent clusters with the same id.*/
		TreeMap trees;
		
		/* Initial symmetric allowed width/heigth/length of new tree roots. */
		double initialTreeDiameter;

		/* Minimal symmetric allowed width/heigth/length of a leaf-nodes (i.e. resolution) .*/
		double minimalCellDiameter;

		/* Flag if trees are allowed to expand themselves to accomodate
		for new objects outside of their inital root sub-space. */
		bool allowRescaling;

		/**
			\brief Calculate the closest inter-hull distance of two spheres.
			\param[in] sphere1 First sphere to be used for distance calculation.
			\param[in] sphere2 Second sphere to be used for distance calculation.
			\returns Closest distance between the two body's hulls. Negative values indicate collision.
		*/
		const double calcSphereToSphereDistance(SimObjPtr sphere1, SimObjPtr sphere2) const;

		/**
			\brief Calculate the closest inter-hull distance of a sphere and a clyinder.
			\param[in] sphere Sphere to be used for distance calculation.
			\param[in] cylinder Cylinder to be used for distance calculation.
			\returns Closest distance between the two body's hulls. Negative values indicate collision.
		*/
		const double calcSphereToCylinderDistance(SimObjPtr sphere, SimObjPtr cylinder) const;

		/**
			\brief Calculate the closest inter-hull distance of two cylinders.
			\param[in] cylinder1 First sphere to be used for distance calculation.
			\param[in] cylinder2 Second sphere to be used for distance calculation.
			\returns Closest distance between the two body's hulls. Negative values indicate collision.
		*/
		const double calcCylinderToCylinderDistance(SimObjPtr cylinder1, SimObjPtr cylinder2) const;
	};
}