#include "..\include\OctTree.h"
#include "..\include\OctTreeNode.h"
#include <queue>

namespace collision {
	namespace octtree {
		
		template <typename T>
		OctTree<T>::OctTree(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter) {
			nodes = NodeArray<T>();
			this->minDiameter = minDiameter;
			root = OctTreeNode<T>::createNode();
			nodes.push_back(root);
			root->setBounds(lower, upper);
			allowResize = false;
		}

		template <typename T>
		OctTree<T>::OctTree(const Bounds& diameter, const Bounds& minDiameter) {
			nodes = NodeArray<T>();
			this->minDiameter = minDiameter;
			root = OctTreeNode<T>::createNode();
			nodes.push_back(root);
			root->setBounds(Bounds(-diameter.x / 2, -diameter.y / 2, -diameter.z / 2), Bounds(diameter.x / 2, diameter.y / 2, diameter.z / 2));
			allowResize = false;
		}

		template <typename T>
		OctTree<T>::OctTree(const double& treeDiameter, const double& minCellDiameter) {
			nodes = NodeArray<T>();
			this->minDiameter = Bounds(minCellDiameter, minCellDiameter, minCellDiameter);
			root = OctTreeNode<T>::createNode();
			nodes.push_back(root);
			root->setBounds(Bounds(-treeDiameter / 2, -treeDiameter / 2, -treeDiameter / 2), Bounds(treeDiameter / 2, treeDiameter / 2, treeDiameter / 2));
			allowResize = false;
		}

		template <typename T>
		TreePtr<T> OctTree<T>::create(const Bounds& lower, const Bounds& upper, const Bounds& minDiameter) {
			return TreePtr<T>(new OctTree<T>(lower, upper, minDiameter));
		}

		template <typename T>
		TreePtr<T> OctTree<T>::create(const Bounds& diameter, const Bounds& minDiameter) {
			return TreePtr<T>(new OctTree<T>(diameter, minDiameter));
		}

		template <typename T>
		TreePtr<T> OctTree<T>::create(const double& treeDiameter, const double& minCellDiameter) {
			return TreePtr<T>(new OctTree<T>(treeDiameter, minCellDiameter));
		}

		template <typename T>
		void OctTree<T>::setAllowResize(const bool resizeOn) {
			this->allowResize = resizeOn;
		}

		template <typename T>
		const NodeArray<T>& OctTree<T>::getNodes() const {
			return nodes;
		}

		template <typename T>
		void OctTree<T>::insertObject(const T& id, const Bounds& lowerBound, const Bounds& upperBound) {
			
			if (allowResize && !root->isInBounds(lowerBound, upperBound)) {
				//resize
				resize(lowerBound, upperBound);
			}
			
			insertObject(root, id, lowerBound, upperBound);
		}

		class mycomparison
		{
			bool reverse;
			double x, y, z;
		public:
			mycomparison(const double& x, const double& y, const double& z ,const bool& revparam = false) :
				x(x), y(y), z(z)
			{
				reverse = revparam;
			}

			template <typename T>
			bool operator() (const NodePtr<T> lhs, const NodePtr<T> rhs) const
			{
			
				double	lhsDist = lhs->calcMinDistance(x, y, z);
				double	rhsDist = rhs->calcMinDistance(x, y, z);
				
				if (reverse) {
					return (lhsDist>rhsDist);
				}
				else {
					return (lhsDist<rhsDist);
				}	
			}
		};

		template <typename T>
		NodePtr<T> OctTree<T>::getNearest(const double& x, const double& y, const double& z) {
			std::priority_queue<NodePtr<T>, std::vector<NodePtr<T>>, mycomparison> queue(mycomparison(x, y, z, true));
			queue.push(root);
			while (!queue.empty()) {
				NodePtr<T> current = queue.top();
				queue.pop();
				if (current->isLeaf()) return current;
				for (unsigned int oct = Octant::first; oct <= Octant::eighth; oct++) {
					if (current->isOctantNull(static_cast<Octant>(oct))) continue;
					queue.push(current->getChild(static_cast<Octant>(oct)));
				}
			}
			return root;
		}

		template <typename T>
		NodePtr<T> OctTree<T>::getNearest(const double& x, const double& y, const double& z, const IdSet<T>& ignoreIDs) {
			if (ignoreIDs.size() < 1) return getNearest(x, y, z);
			std::priority_queue<NodePtr<T>, std::vector<NodePtr<T>>, mycomparison> queue(mycomparison(x, y, z, true));
			queue.push(root);
			while (!queue.empty()) {
				NodePtr<T> current = queue.top();
				queue.pop();
				bool ignore = false;
				unsigned int found = 0;
				for (auto id : ignoreIDs) {
					if (current->getIds().find(id) != current->getIds().end()) found++;
				}
				if (found == current->getIds().size()) ignore = true;
				if (current->isLeaf() && !ignore) return current;
				for (unsigned int oct = Octant::first; oct <= Octant::eighth; oct++) {
					if (current->isOctantNull(static_cast<Octant>(oct))) continue;
					queue.push(current->getChild(static_cast<Octant>(oct)));
				}
			}
			return root;
		}

		template <typename T>
		bool OctTree<T>::insertObject(NodePtr<T> node, const T& id, const Bounds& lowerBound, const Bounds& upperBound) {
			Bounds diameter = node->getDiameter();
			// recursion anchor
			if ( (diameter.x <= minDiameter.x) || (diameter.y <= minDiameter.y) || (diameter.z <= minDiameter.z) ) {
				node->addId(id);
				return true;
			} else {
				// -> doesn't fit in children -> split up !
				// 1 find all involved octants
				unordered_set<Octant> involvedOctants = node->findInvolvedOctants(lowerBound, upperBound);
				// 2 create missing octants
				for (auto oct : involvedOctants) {
					if (oct == Octant::none) continue;
					if (node->isOctantNull(oct)) {
						Bounds lb = node->calcChildLowerBounds(oct);
						Bounds ub = node->calcChildUpperBounds(oct);
						makeNewOctant(node, lb, ub, oct);
					}
				}
				// 3 split bounds up 
				vector<Bounds> ubs;
				vector<Bounds> lbs;
				for (auto oct : involvedOctants) {
					if (oct == Octant::none) continue;
					lbs.push_back(node->calcPartialLowerBounds(oct, lowerBound));
					ubs.push_back(node->calcPartialUpperBounds(oct, upperBound));
					Bounds u = node->calcPartialUpperBounds(oct, upperBound);
					Bounds l = node->calcPartialLowerBounds(oct, lowerBound);
				}
				// 4 call multiple insertNodes recursively
				unsigned int c = 0;
				for (auto oct : involvedOctants) {
					if (oct == Octant::none) continue;
					insertObject(node->getChild(oct), id, lbs[c], ubs[c]);
					c++;
				}
			return true;
			}
		}

		template <typename T>
		void OctTree<T>::makeNewOctant(NodePtr<T> parent, const Bounds& lowerBound, const Bounds& upperBound, const Octant& octant) {
			NodePtr<T> oct = OctTreeNode<T>::createNode();
			nodes.push_back(oct);
			oct->setBounds(lowerBound, upperBound);
			oct->setParent(parent);
			parent->setChild(oct, octant);
		}

		template <typename T>
		void OctTree<T>::resize(const Bounds& lowerBound, const Bounds& upperBound) {
			unsigned int steps = 0;
			while (!root->isInBounds(lowerBound, upperBound) && steps < MAX_RESIZE_STEPS) {
				steps++;
				NodePtr<T> first = root->getChild(Octant::first);
				NodePtr<T> second = root->getChild(Octant::second);
				NodePtr<T> third = root->getChild(Octant::third);
				NodePtr<T> fourth = root->getChild(Octant::fourth);
				NodePtr<T> fifth = root->getChild(Octant::fifth);
				NodePtr<T> sixth = root->getChild(Octant::sixth);
				NodePtr<T> seventh = root->getChild(Octant::seventh);
				NodePtr<T> eighth = root->getChild(Octant::eighth);

				Bounds rlb = root->getLowerBounds();
				Bounds rub = root->getUpperBounds();
				root->setBounds( Bounds(rlb.x*2, rlb.y*2, rlb.z*2), Bounds(rub.x * 2, rub.y * 2, rub.z * 2) );

				for (unsigned int i = Octant::first; i <= Octant::eighth; i++) {
					Bounds lb = root->calcChildLowerBounds(static_cast<Octant>(i));
					Bounds ub = root->calcChildUpperBounds(static_cast<Octant>(i));
					makeNewOctant(root, lb, ub, static_cast<Octant>(i));
				}
				root->getChild(Octant::first)->setChild(first, Octant::seventh);
				root->getChild(Octant::second)->setChild(second, Octant::eighth);
				root->getChild(Octant::third)->setChild(third, Octant::fifth);
				root->getChild(Octant::fourth)->setChild(fourth, Octant::sixth);
				root->getChild(Octant::fifth)->setChild(fifth, Octant::third);
				root->getChild(Octant::sixth)->setChild(sixth, Octant::fourth);
				root->getChild(Octant::seventh)->setChild(seventh, Octant::first);
				root->getChild(Octant::eighth)->setChild(seventh, Octant::second);
			}
		}

		template class OctTree<double>;
		template class OctTree<unsigned int>;
	}
}