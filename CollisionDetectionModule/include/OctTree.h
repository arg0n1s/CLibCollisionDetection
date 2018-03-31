#pragma once
#undef max
#include <memory>
#include <vector>
#include <unordered_set>
#include <limits>
#include <string>

namespace collision {
	namespace octtree {

		// Aliases for used namespaces
		using std::shared_ptr;
		using std::weak_ptr;
		using std::vector;
		using std::unordered_set;

		/**
			\brief Helper struct to remove the number of parameters when defining 
			spatial boundaries (aka. Bounding-Box) of a shape that should be inserted
			into the OctTree.
			To define boundaries two Bounds-objects are required, one for defining the
			upper bound and the other for defining the lower bounds of an object.
		*/
		struct Bounds {

			/* Karthesian coordinates of the lower or upper bound.*/
			double x, y, z;
			
			/**
				\brief Default-Constructor
			*/
			Bounds() {};

			/**
				\brief Construct a Bounds-object representing the lower or upper bounds
				of a shape.
				\param[in] x Max. or min. boundary in x direction of the shape.
				\param[in] y Max. or min. boundary in y direction of the shape.
				\param[in] z Max. or min. boundary in z direction of the shape.
			*/
			Bounds(const double& x, const double& y, const double& z) : x(x), y(y), z(z) {};
		};

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

			unsigned int index;

			/* Id container of any shapes/bodies/entities this node contains. */
			IdSet<T> ids;

			/**
				\brief Default-Constructor
			*/
			OctTreeNode();

		};

		// Forward declaration to deal with circular dependencies.
		template <typename T>
		class OctTree;

		// Aliases for used namespaces
		template <typename T>
		using NodeArray = std::vector <NodePtr<T>>;
		template <typename T>
		using TreePtr = shared_ptr<OctTree<T>>;

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
			void insertNode(const T& id, const Bounds& lowerBound, const Bounds& upperBound);
			double getNearestDistance(const double& x, const double& y, const double& z);

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
			bool insertNode(NodePtr<T> node, const T& id, const Bounds& lowerBound, const Bounds& upperBound);
		};
	}
}
