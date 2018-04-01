#pragma once
#include <vector>
#include <memory>
#include <unordered_set>
#include "..\include\Bounds.h"

namespace collision {
	namespace octtree {

		// Aliases for used namespaces
		using std::shared_ptr;
		using std::weak_ptr;
		using std::vector;
		using std::unordered_set;

		/* Positive Infinity Constant */
		static const double POS_INF = std::numeric_limits<double>::max();

		/* Negative Infinity Constant */
		static const double NEG_INF = -std::numeric_limits<double>::max();

		/* Maximum number of resize iterations an OctTree is allowed to undergo. */
		static const unsigned int MAX_RESIZE_STEPS = 10;

		namespace octant {

			/**
			/brief Enumerator used to define equally sized sectors of real vector space aka. Octants.
			*/
			enum Octant {
				first, second, third, fourth,
				fifth, sixth, seventh, eighth,
				none
			};
		}

		// Typedef for ease of use
		typedef octant::Octant Octant;

		// Forward declaration to deal with circular dependencies.
		template <typename T>
		class OctTreeNode;

		// Aliases for used namespaces
		template <typename T>
		using NodePtr = shared_ptr<OctTreeNode<T>>;
		template <typename T>
		using WeakNodePtr = weak_ptr<OctTreeNode<T>>;
		template <typename T>
		using Octants = std::vector<WeakNodePtr<T>>;
		template <typename T>
		using IdSet = std::unordered_set<T>;

		/**
		\brief Objects of this class are used to represent nodes in an OctTree
		and therefore carry information (e.g. IDs) about shapes/bodies/entities that may reside
		inside the subspace a node governs over.
		Nodes know their center's coordinates, octant, boundaries, parent- and child-nodes.
		*/
		template <typename T>
		class OctTreeNode {
		public:

			/**
			\brief Call to instantiate a new OctTreeNode-object.
			\returns Returns a smart pointer to the new object on the heap.
			\note This is the only way to "make" an OctTreeNode-object, i.e. Constructor is private!
			*/
			static NodePtr<T> createNode();

			/**
			\brief Check if this node is empty, i.e. does not contain any shapes/bodies/entities.
			\returns True if this node contains any shapes/bodies/entities.
			*/
			const bool isEmpty() const;

			/**
			\brief Check if this node does not have child-nodes, i.e. is a leaf.
			\returns True if this node is a leaf.
			*/
			const bool isLeaf() const;

			/**
			\brief Check if this node does not have a parent-node, i.e. is root.
			\returns True if this node is root.
			*/
			const bool isRoot() const;

			/**
			\brief Check if this node governs over a valid octant, i.e. octant is not null.
			\returns True if this nodes octant is not null.
			*/
			const bool isOctantNull(const Octant& octant) const;

			/**
			\brief Set one of 8 child nodes of this node.
			\param[in] child Smart pointer to a node that will become a child of this node.
			\param[in] octant Sub-Octant which this child will cover.
			*/
			void setChild(NodePtr<T> child, const Octant& octant);

			/**
			\brief Get the child node that covers the given octant.
			\param[in] octant Sub-Octant coverd by the requested child.
			returns Requested child node.
			*/
			NodePtr<T> getChild(const Octant& octant);

			/**
			\brief Set the parent node of this node.
			\param[in] parent Smart pointer to a node that will become the parent of this node.
			*/
			void setParent(NodePtr<T> parent);

			/**
			\brief Set the spatial boundaries of this node.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			*/
			void setBounds(const Bounds& lowerBound, const Bounds& upperBound);

			/**
			\brief Get the x-coordinate of this nodes origin.
			\returns x-coordinate
			*/
			const double& getX() const;

			/**
			\brief Get the y-coordinate of this nodes origin.
			\returns y-coordinate
			*/
			const double& getY() const;

			/**
			\brief Get the z-coordinate of this nodes origin.
			\returns z-coordinate
			*/
			const double& getZ() const;

			/**
			\brief Get the lower spatial boundaries of this node.
			\returns Coordinates of the lower bound in real vector space.
			*/
			const Bounds& getLowerBounds() const;

			/**
			\brief Get the upper spatial boundaries of this node.
			\returns Coordinates of the upper bound in real vector space.
			*/
			const Bounds& getUpperBounds() const;

			/**
			\brief Get the diameter of upper an lower bounds.
			\example diameter(upperX, lowerX) = absolute(upperX - lowerX)
			\returns Diameter values of x-, y-, z-boundaries as a Bounds-object.
			*/
			const Bounds getDiameter() const;

			/**
			\brief Check if a bounding-box is inside this node's boundaries.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns True if the given bounding-box fits inside this node's boundaries.
			*/
			const bool isInBounds(const Bounds& lowerBound, const Bounds& upperBound) const;

			/**
			\brief Check if a point is inside this node's boundaries.
			\param[in] x X-Coordinate of the given query-point
			\param[in] y Y-Coordinate of the given query-point
			\param[in] z Z-Coordinate of the given query-point
			\returns True if the given query-point fits inside this node's boundaries.
			*/
			const bool isInBounds(const double& x, const double& y, const double& z) const;

			/**
			\brief Check if a bounding-box is inside one of this node's child boundaries.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns True if the given bounding-box fits inside one of this node's child boundaries.
			*/
			const bool boundsFitInChildNodes(const Bounds& lowerBound, const Bounds& upperBound) const;

			/**
			\brief Find the child-octant in which the given bounding-box would fit in.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns Ocant enumerator of the fitting sub-octant.
			*/
			const Octant calcOctantOfBounds(const Bounds& lowerBound, const Bounds& upperBound) const;

			/**
			\brief Find the child-octant in which the given query-point would fit in.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns Ocant enumerator of the fitting sub-octant.
			*/
			const Octant calcOctantOfPoint(const double& x, const double& y, const double& z) const;

			/**
			\brief Find all of the child-octants the given bounding-box would have to occupy.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns Set of possibly affected sub-octants.
			*/
			const unordered_set<Octant> findInvolvedOctants(const Bounds& lowerBound, const Bounds& upperBound) const;

			/**
			\brief Calculate the lower spatial boundaries of sub-octant in this node.
			\param[in] octant Sub-octant aka. child-node of interest.
			\returns Coordinates of the lower bound in real vector space.
			*/
			const Bounds calcChildLowerBounds(const Octant& octant) const;

			/**
			\brief Calculate the upper spatial boundaries of sub-octant in this node.
			\param[in] octant Sub-octant aka. child-node of interest.
			\returns Coordinates of the upper bound in real vector space.
			*/
			const Bounds calcChildUpperBounds(const Octant& octant) const;

			/**
			\brief Calculate the lower bounds of the overlap of a given bounding-box and the given sub-octant.
			\param[in] octant Sub-octant aka. child-node of interest.
			\param[in] lowerBound Coordinates of the lower bound in real vector space.
			\returns Coordinates of the upper bound of the overlap in real vector space.
			*/
			const Bounds calcPartialLowerBounds(const Octant& octant, const Bounds& lowerBound) const;

			/**
			\brief Calculate the upper bounds of the overlap of a given bounding-box and the given sub-octant.
			\param[in] octant Sub-octant aka. child-node of interest.
			\param[in] upperBound Coordinates of the upper bound in real vector space.
			\returns Coordinates of the upper bound of the overlap in real vector space.
			*/
			const Bounds calcPartialUpperBounds(const Octant& octant, const Bounds& upperBound) const;

			/**
			\brief Calculate the minimum distance of a given query-point to this node's bounding-box surface.
			\param[in] x X-Coordinate of the given query-point
			\param[in] y Y-Coordinate of the given query-point
			\param[in] z Z-Coordinate of the given query-point
			\returns Distance of the query-point to the BBox surface, 0 if inside the BBbox.
			*/
			const double calcMinDistance(const double& x, const double& y, const double& z) const;

			/**
			\brief Add unique identifiers of shapes/bodies/entities that may reside in this node.
			\param[in] id unique identifier of an entity
			*/
			void addId(const T& id);

			/**
			\brief Get the unique identifiers of shapes/bodies/entities that reside in this node.
			\returns set of id unique identifiers of entity in this node
			*/
			const IdSet<T>& getIds() const;

			/**
			\brief Packs the node's internal information into a formatted readable string.
			*/
			const std::string toString() const;

		protected:

			/* This nodes parent-node */
			WeakNodePtr<T> parent;

			/* This nodes child-nodes, aka. sub-octants. */
			Octants<T> octants;

			/* Flag if this node is root. */
			bool root;

			/* Flag if this node is empty, i.e. does not contain any shapes/bodies/entities. */
			bool empty;

			/* Flag if this node is a leaf, i.e. does not have child-nodes. */
			bool leaf;

			/* Lower and upper spatial bounds of this node, aka. bounding-box. */
			Bounds lower, upper;

			/* Coordinates of this nodes origin/center. */
			double x, y, z;

			/* Diameter of this node's bounds in x, y and z direction, aka. width, height, length. */
			double width, height, length;

			/* Id container of any shapes/bodies/entities this node contains. */
			IdSet<T> ids;

			/**
			\brief Default-Constructor
			*/
			OctTreeNode();

		};

	}
}
