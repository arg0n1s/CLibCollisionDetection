#pragma once
#undef max
#include <memory>
#include <vector>
#include <unordered_set>
#include "..\include\Bounds.h"

namespace collision {
	namespace octtree {

		// Forward declaration to deal with circular dependencies.
		template <typename T>
		class OctTreeNode;

		// Forward declaration to remove unnecessary includes.
		template <typename T>
		class OctTree;

		namespace octant {
			// Forward declaration to remove unnecessary includes.
			enum Octant;
		}
		// Typedef for ease of use
		typedef octant::Octant Octant;

		// Aliases for used namespaces
		using std::shared_ptr;
		using std::unordered_set;

		template <typename T>
		using NodePtr = shared_ptr<OctTreeNode<T>>;
		template <typename T>
		using NodeArray = std::vector <NodePtr<T>>;
		template <typename T>
		using TreePtr = shared_ptr<OctTree<T>>;
		template <typename T>
		using IdSet = std::unordered_set<T>;

		/**
			\brief Objects of this class contain a tree structure called OctTree, that divides real vector space (R^3)
			into equally spaced sub-spaces (aka. octants).
			A node in this tree may contain complete 3D-objects, parts of it or 3D-points.
			New objects may be inserted by using their bounding-box (for 3D objects) or their coordinates (for 3D-points).
			If an object that lies outside of a tree's outer boundaries is inserted, the tree will be expanded until the obect fits.
			Expanding the tree means, that the root diameter is doubled (up to 10 times) and child nodes are re-inserted.
			The OctTree's main function is to do a nearest neigbor search for query-points, i.e. find the nearest collection of
			objects/points in this tree to the given query-point. This may be used to implement an efficient and
			fast collision checking algorithm.
			\note Only leaf-nodes may contain objects in this OctTree implementation, but one object may occupy more than one
			leaf-node if its boundaries overlap with other leafs.
		*/
		template <typename T>
		class OctTree {
		public:

			/**
				\brief Call to instantiate a new OctTree-object.
				\param[in] lower Initial lower spatial bounds of this tree's root.
				\param[in] upper Initial upper spatial bounds of this tree's root.
				\param[in] minDiameter Minimal allowed symmetric width/heigth/length of a leaf-node (i.e. resolution).
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" an OctTree-object, i.e. Constructor is private!
			*/
			static TreePtr<T> create(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter);

			/**
				\brief Call to instantiate a new OctTree-object.
				\param[in] diameter Initial symmetric spatial bounds of this tree's root.
				\param[in] minDiameter Minimal symmetric allowed width/heigth/length of a leaf-node (i.e. resolution).
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" an OctTree-object, i.e. Constructor is private!
			*/
			static TreePtr<T> create(const Bounds& diameter, const Bounds& minDiameter);

			/**
				\brief Call to instantiate a new OctTree-object.
				\param[in] treeDiameter Initial symmetric allowed width/heigth/length of this tree's root.
				\param[in] minDiameter Minimal symmetric allowed width/heigth/length of a leaf-node (i.e. resolution).
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" an OctTree-object, i.e. Constructor is private!
			*/
			static TreePtr<T> create(const double& treeDiameter, const double& minCellDiameter);

			/**
				\brief Set the resize allowed flag for this tree.
				\param[in] resizeOn If true, this tree is allowed to expand itself to accomodate 
				for new objects outside of the inital root sub-space.
			*/
			void setAllowResize(const bool resizeOn);


			/**
				\brief Insert a new object with its unique identifier and bounding box into this tree.
				\param[in] id unique identifier of the inserted object.
				\param[in] lowerBound Coordinates of the object's BBox lower bound in real vector space.
				\param[in] upperBound Coordinates of the object's BBox upper bound in real vector space.
			*/
			void insertObject(const T& id, const Bounds& lowerBound, const Bounds& upperBound);

			/**
				\brief Get the node with the bounding box that has the nearest distance to the query-point.
				\param[in] x X-Coordinate of the given query-point
				\param[in] y Y-Coordinate of the given query-point
				\param[in] z Z-Coordinate of the given query-point
				\returns Closest node of the tree that is not empty.
				\note If the tree is empty root is returned.
			*/
			NodePtr<T> getNearest(const double& x, const double& y, const double& z);

			/**
				\brief Get the node with the bounding box that has the nearest distance to the query-point.
				But ignore any nodes that only contain ids that should be ignored.
				\param[in] x X-Coordinate of the given query-point
				\param[in] y Y-Coordinate of the given query-point
				\param[in] z Z-Coordinate of the given query-point
				\param[in] ignoreIDs Set of IDs that should be ignored while finding the nearest node.
				\returns Closest node of the tree that is not empty.
				\note If the tree is empty root is returned.
			*/
			NodePtr<T> getNearest(const double& x, const double& y, const double& z, const IdSet<T>& ignoreIDs);

			/**
				\brief Get all nodes of this tree.
				\returns Array of this tree's nodes.
			*/
			const NodeArray<T>& getNodes() const;

		private:

			/* Array of this tree's nodes. */
			NodeArray<T> nodes;

			/* This tree's root-node. */
			NodePtr<T> root;

			/* Minimal symmetric diameter of this tree's leaf-nodes. */
			Bounds minDiameter;

			/* Flag if this tree is allowed to expand itself to accomodate 
				for new objects outside of the inital root sub-space. */
			bool allowResize;

			/**
				\brief Construct a new OctTree-object.
				\param[in] lower Initial lower spatial bounds of this tree's root.
				\param[in] upper Initial upper spatial bounds of this tree's root.
				\param[in] minDiameter Minimal allowed symmetric width/heigth/length of a leaf-node (i.e. resolution).
			*/
			OctTree(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter);

			/**
				\brief Construct a new OctTree-object.
				\param[in] diameter Initial symmetric spatial bounds of this tree's root.
				\param[in] minDiameter Minimal symmetric allowed width/heigth/length of a leaf-node (i.e. resolution).
			*/
			OctTree(const Bounds& diameter, const Bounds& minDiameter);

			/**
				\brief Construct a new OctTree-object.
				\param[in] treeDiameter Initial symmetric allowed width/heigth/length of this tree's root.
				\param[in] minDiameter Minimal symmetric allowed width/heigth/length of a leaf-node (i.e. resolution).
			*/
			OctTree(const double& treeDiameter, const double& minCellDiameter);

			/**
				\brief Create a new child-node that covers a sub-octant of its parent-node.
				\param[in] parent Smart pointer to the parent node of the new child.
				\param[in] lower Lower spatial bounds of this new child.
				\param[in] upper Upper spatial bounds of this new child.
				\param[in] octant Sub-ocant the new child should cover.
			*/
			void makeNewOctant(NodePtr<T> parent, const Bounds& lowerBound, const Bounds& upperBound, const Octant& octant);

			/**
				\brief Resize this tree to the new given root-bounds
				\param[in] lower Lower spatial bounds of the new tree.
				\param[in] upper Upper spatial bounds of the new tree.
			*/
			void resize(const Bounds& lowerBound, const Bounds& upperBound);

			/**
				\brief Recursively insert a new object with its unique identifier and bounding box into this tree.
				\param[in] node Callee responsible for this recursion layer.
				\param[in] id unique identifier of the inserted object.
				\param[in] lowerBound Coordinates of the object's BBox lower bound in real vector space.
				\param[in] upperBound Coordinates of the object's BBox upper bound in real vector space.
			*/
			bool insertObject(NodePtr<T> node, const T& id, const Bounds& lowerBound, const Bounds& upperBound);
		};
	}
}
