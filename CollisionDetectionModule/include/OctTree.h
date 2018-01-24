#pragma once
#undef max
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <string>

namespace collision {
	namespace octtree {
		using std::shared_ptr;
		using std::weak_ptr;
		using std::vector;
		using std::unordered_set;
		struct Bounds {
			double x, y, z;
			Bounds() {};
			Bounds(const double& x, const double& y, const double& z) : x(x), y(y), z(z) {};
		};
		static const double POS_INF = std::numeric_limits<double>::max();
		static const double NEG_INF = -std::numeric_limits<double>::max();

		namespace octant {
			enum Octant {
				first, second, third, fourth,
				fifth, sixth, seventh, eighth,
				none
			};
		}
		typedef octant::Octant Octant;

		template <typename T>
		class OctTreeNode;

		template <typename T>
		using NodePtr = shared_ptr<OctTreeNode<T>>;
		template <typename T>
		using WeakNodePtr = weak_ptr<OctTreeNode<T>>;
		template <typename T>
		using Octants = std::vector<WeakNodePtr<T>>;
		template <typename T>
		using IdSet = std::unordered_set<T>;

		template <typename T>
		class OctTreeNode {
		public:
			static NodePtr<T> createNode();

			const bool isEmpty() const;
			const bool isLeaf() const;
			const bool isRoot() const;
			const bool isOctantNull(const Octant& octant) const;

			void setChild(NodePtr<T> child, const Octant& octant);
			NodePtr<T> getChild(const Octant& octant);

			void setParent(NodePtr<T> parent);
			void setBounds(const Bounds& lowerBound, const Bounds& upperBound);

			const double& getX() const;
			const double& getY() const;
			const double& getZ() const;

			const Bounds& getLowerBounds() const;
			const Bounds& getUpperBounds() const;
			const Bounds getDiameter() const;

			const bool isInBounds(const Bounds& lowerBound, const Bounds& upperBound) const;
			const bool isInBounds(const double& x, const double& y, const double& z) const;
			const bool boundsFitInChildNodes(const Bounds& lowerBound, const Bounds& upperBound) const;

			const Octant calcOctantOfBounds(const Bounds& lowerBound, const Bounds& upperBound) const;
			const Octant calcOctantOfPoint(const double& x, const double& y, const double& z) const;
			const unordered_set<Octant> findInvolvedOctants(const Bounds& lowerBound, const Bounds& upperBound) const;

			const Bounds calcChildLowerBounds(const Octant& octant) const;
			const Bounds calcChildUpperBounds(const Octant& octant) const;

			const Bounds calcPartialLowerBounds(const Octant& octant, const Bounds& lowerBound) const;
			const Bounds calcPartialUpperBounds(const Octant& octant, const Bounds& upperBound) const;
			
			const double calcMinDistance(const double& x, const double& y, const double& z) const;

			void addId(const T& id);
			const IdSet<T>& getIds() const;

			const std::string toString() const;

		protected:
			WeakNodePtr<T> parent;
			Octants<T> octants;
			bool root;
			bool empty;
			bool leaf;
			Bounds lower, upper;
			double x, y, z;
			double width, height, length;
			unsigned int index;
			IdSet<T> ids;

			OctTreeNode();

		};

		template <typename T>
		class OctTree;

		template <typename T>
		using NodeArray = std::vector <NodePtr<T>>;
		template <typename T>
		using NodeMap = std::unordered_map <T, NodePtr<T>>;
		template <typename T>
		using TreePtr = shared_ptr<OctTree<T>>;

		template <typename T>
		class OctTree {
		public:
			static TreePtr<T> create(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter);

			void insertNode(const T& id, const Bounds& lowerBound, const Bounds& upperBound);
			double getNearestDistance(const double& x, const double& y, const double& z);
			NodePtr<T> getNearest(const double& x, const double& y, const double& z);

			const NodeArray<T>& getNodes() const;

		protected:
			NodeArray<T> nodes;
			NodeMap<T> mappedNodes;
			NodePtr<T> root;
			Bounds minDiameter;

			OctTree(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter);

			void makeNewOctant(NodePtr<T> parent, const Bounds& lowerBound, const Bounds& upperBound, const Octant& octant);
			
			bool insertNode(NodePtr<T> node, const T& id, const Bounds& lowerBound, const Bounds& upperBound);
		};
	}
}
