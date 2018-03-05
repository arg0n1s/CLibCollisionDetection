#include "..\include\OctTree.h"
#include <sstream>
#include <iostream>
#include <queue>

namespace collision {
	namespace octtree {

		template <typename T>
		NodePtr<T> OctTreeNode<T>::createNode() {
			return NodePtr<T>(new OctTreeNode<T>());
		}

		template <typename T>
		const bool OctTreeNode<T>::isEmpty() const {
			return empty;
		}

		template <typename T>
		const bool OctTreeNode<T>::isLeaf() const {
			return leaf;
		}

		template <typename T>
		const bool OctTreeNode<T>::isRoot() const {
			return root;
		}

		template <typename T>
		const bool OctTreeNode<T>::isOctantNull(const Octant& octant) const {
			NodePtr<T> oct = octants[octant].lock();
			return (oct == nullptr);
		}

		template <typename T>
		void OctTreeNode<T>::setChild(NodePtr<T> child, const Octant& octant) {
			octants[octant] = child;
			leaf = false;
		}

		template <typename T>
		NodePtr<T> OctTreeNode<T>::getChild(const Octant& octant) {
			return octants[octant].lock();
		}

		template <typename T>
		void OctTreeNode<T>::setParent(NodePtr<T> parent) {
			this->parent = parent;
			root = false;
		}

		template <typename T>
		void OctTreeNode<T>::setBounds(const Bounds& lowerBound, const Bounds& upperBound) {
			lower = lowerBound;
			upper = upperBound;

			x = lower.x + (upper.x - lower.x) / 2.0;
			y = lower.y + (upper.y - lower.y) / 2.0;
			z = lower.z + (upper.z - lower.z) / 2.0;

			width = std::fabs(upper.x - lower.x);
			height = std::fabs(upper.y - lower.y);
			length = std::fabs(upper.z - lower.z);
		}

		template <typename T>
		const double& OctTreeNode<T>::getX() const {
			return x;
		}

		template <typename T>
		const double& OctTreeNode<T>::getY() const {
			return y;
		}

		template <typename T>
		const double& OctTreeNode<T>::getZ() const {
			return z;
		}

		template <typename T>
		const Bounds& OctTreeNode<T>::getLowerBounds() const {
			return lower;
		}

		template <typename T>
		const Bounds& OctTreeNode<T>::getUpperBounds() const {
			return upper;
		}

		template <typename T>
		const Bounds OctTreeNode<T>::getDiameter() const {
			return Bounds(width, height, length);
		}

		template <typename T>
		const bool OctTreeNode<T>::isInBounds(const Bounds& lowerBound, const Bounds& upperBound) const {
			if (lowerBound.x < lower.x) return false;
			if (lowerBound.y < lower.y) return false;
			if (lowerBound.z < lower.z) return false;
			if (upperBound.x > upper.x) return false;
			if (upperBound.y > upper.y) return false;
			if (upperBound.z > upper.z) return false;
			return true;
		}

		template <typename T>
		const bool OctTreeNode<T>::isInBounds(const double& x, const double& y, const double& z) const {
			if (lower.x > x || x > upper.x) return false;
			if (lower.y > y || y > upper.y) return false;
			if (lower.z > z || z > upper.z) return false;
			return true;
		}

		template <typename T>
		const bool OctTreeNode<T>::boundsFitInChildNodes(const Bounds& lowerBound, const Bounds& upperBound) const {
			if (width < std::fabs(upperBound.x - lowerBound.x)) return false;
			if (height < std::fabs(upperBound.y - lowerBound.y)) return false;
			if (length < std::fabs(upperBound.z - lowerBound.z)) return false;
			Octant octant = calcOctantOfBounds(lowerBound, upperBound);
			if (octant == Octant::none) return false;
			return true;
		}

		template <typename T>
		const Bounds OctTreeNode<T>::calcChildLowerBounds(const Octant& octant) const {
			switch (octant) {
				case Octant::first: {
					//first octant (+,+,+)
					return Bounds(x, y, z);
				}
				case Octant::second: {
					//second octant (-,+,+)
					return Bounds(lower.x, y, z);
				}
				case Octant::third: {
					//third octant (-,-,+)
					return Bounds(lower.x, lower.y, z);
				}
				case Octant::fourth: {
					//fourth octant (+,-,+)
					return Bounds(x, lower.y, z);
				}
				case Octant::fifth: {
					//fifth octant (+,+,-)
					return Bounds(x, y, lower.z);
				}
				case Octant::sixth: {
					//sixth octant (-,+,-)
					return Bounds(lower.x, y, lower.z);
				}
				case Octant::seventh: {
					//seventh octant (-,-,-)
					return Bounds(lower.x, lower.y, lower.z);
				}
				case Octant::eighth: {
					//eigth octant (+,-,-)
					return Bounds(x, lower.y, lower.z);
				}
				default: {
					return Bounds();
				}

			}

		}

		template <typename T>
		const Bounds OctTreeNode<T>::calcChildUpperBounds(const Octant& octant) const {
			switch (octant) {
				case Octant::first: {
					//first octant (+,+,+)
					return Bounds(upper.x, upper.y, upper.z);
				}
				case Octant::second: {
					//second octant (-,+,+)
					return Bounds(x, upper.y, upper.z);
				}
				case Octant::third: {
					//third octant (-,-,+)
					return Bounds(x, y, upper.z);
				}
				case Octant::fourth: {
					//fourth octant (+,-,+)
					return Bounds(upper.x, y, upper.z);
				}
				case Octant::fifth: {
					//fifth octant (+,+,-)
					return Bounds(upper.x, upper.y, z);
				}
				case Octant::sixth: {
					//sixth octant (-,+,-)
					return Bounds(x, upper.y, z);
				}
				case Octant::seventh: {
					//seventh octant (-,-,-)
					return Bounds(x, y, z);
				}
				case Octant::eighth: {
					//eigth octant (+,-,-)
					return Bounds(upper.x, y, z);
				}
				default: {
					return Bounds();
				}

			}

		}

		template <typename T>
		const Bounds OctTreeNode<T>::calcPartialLowerBounds(const Octant& octant, const Bounds& lowerBound) const {
			double lbx, lby, lbz;

			switch (octant) {
				case Octant::first: {
					//first octant (+,+,+)
					lbx = (lowerBound.x > x) ? lowerBound.x : x;
					lby = (lowerBound.y > y) ? lowerBound.y : y;
					lbz = (lowerBound.z > z) ? lowerBound.z : z;
					break;
				}
				case Octant::second: {
					//second octant (-,+,+)
					lbx = (lowerBound.x > lower.x) ? lowerBound.x : lower.x;
					lby = (lowerBound.y > y) ? lowerBound.y : y;
					lbz = (lowerBound.z > z) ? lowerBound.z : z;
					break;
				}
				case Octant::third: {
					//third octant (-,-,+)
					lbx = (lowerBound.x > lower.x) ? lowerBound.x : lower.x;
					lby = (lowerBound.y > lower.y) ? lowerBound.y : lower.y;
					lbz = (lowerBound.z > z) ? lowerBound.z : z;
					break;
				}
				case Octant::fourth: {
					//fourth octant (+,-,+)
					lbx = (lowerBound.x > x) ? lowerBound.x : x;
					lby = (lowerBound.y > lower.y) ? lowerBound.y : lower.y;
					lbz = (lowerBound.z > z) ? lowerBound.z : z;
					break;
				}
				case Octant::fifth: {
					//fifth octant (+,+,-)
					lbx = (lowerBound.x > x) ? lowerBound.x : x;
					lby = (lowerBound.y > y) ? lowerBound.y : y;
					lbz = (lowerBound.z > lower.z) ? lowerBound.z : lower.z;
					break;
				}
				case Octant::sixth: {
					//sixth octant (-,+,-)
					lbx = (lowerBound.x > lower.x) ? lowerBound.x : lower.x;
					lby = (lowerBound.y > y) ? lowerBound.y : y;
					lbz = (lowerBound.z > lower.z) ? lowerBound.z : lower.z;
					break;
				}
				case Octant::seventh: {
					//seventh octant (-,-,-)
					lbx = (lowerBound.x > lower.x) ? lowerBound.x : lower.x;
					lby = (lowerBound.y > lower.y) ? lowerBound.y : lower.y;
					lbz = (lowerBound.z > lower.z) ? lowerBound.z : lower.z;
					break;
				}
				case Octant::eighth: {
					//eigth octant (+,-,-)
					lbx = (lowerBound.x > x) ? lowerBound.x : x;
					lby = (lowerBound.y > lower.y) ? lowerBound.y : lower.y;
					lbz = (lowerBound.z > lower.z) ? lowerBound.z : lower.z;
					break;
				}
				default: {
					lbx = lby = lbz = 0;
				}

			}
			return Bounds(lbx, lby, lbz);

		}

		template <typename T>
		const Bounds OctTreeNode<T>::calcPartialUpperBounds(const Octant& octant, const Bounds& upperBound) const {
			double ubx, uby, ubz;

			switch (octant) {
				case Octant::first: {
					//first octant (+,+,+)
					ubx = (upperBound.x < upper.x) ? upperBound.x : upper.x;
					uby = (upperBound.y < upper.y) ? upperBound.y : upper.y;
					ubz = (upperBound.z < upper.z) ? upperBound.z : upper.z;
					break;
				}
				case Octant::second: {
					//second octant (-,+,+)
					ubx = (upperBound.x < x) ? upperBound.x : x;
					uby = (upperBound.y < upper.y) ? upperBound.y : upper.y;
					ubz = (upperBound.z < upper.z) ? upperBound.z : upper.z;
					break;
				}
				case Octant::third: {
					//third octant (-,-,+)
					ubx = (upperBound.x < x) ? upperBound.x : x;
					uby = (upperBound.y < y) ? upperBound.y : y;
					ubz = (upperBound.z < upper.z) ? upperBound.z : upper.z;
					break;
				}
				case Octant::fourth: {
					//fourth octant (+,-,+)
					ubx = (upperBound.x < upper.x) ? upperBound.x : upper.x;
					uby = (upperBound.y < y) ? upperBound.y : y;
					ubz = (upperBound.z < upper.z) ? upperBound.z : upper.z;
					break;
				}
				case Octant::fifth: {
					//fifth octant (+,+,-)
					ubx = (upperBound.x < upper.x) ? upperBound.x : upper.x;
					uby = (upperBound.y < upper.y) ? upperBound.y : upper.y;
					ubz = (upperBound.z < z) ? upperBound.z : z;
					break;
				}
				case Octant::sixth: {
					//sixth octant (-,+,-)
					ubx = (upperBound.x < x) ? upperBound.x : x;
					uby = (upperBound.y < upper.y) ? upperBound.y : upper.y;
					ubz = (upperBound.z < z) ? upperBound.z : z;
					break;
				}
				case Octant::seventh: {
					//seventh octant (-,-,-)
					ubx = (upperBound.x < x) ? upperBound.x : x;
					uby = (upperBound.y < y) ? upperBound.y : y;
					ubz = (upperBound.z < z) ? upperBound.z : z;
					break;
				}
				case Octant::eighth: {
					//eigth octant (+,-,-)
					ubx = (upperBound.x < upper.x) ? upperBound.x : upper.x;
					uby = (upperBound.y < y) ? upperBound.y : y;
					ubz = (upperBound.z < z) ? upperBound.z : z;
					break;
				}
				default: {
					ubx = uby = ubz = 0;
				}
			}

		    return Bounds(ubx, uby, ubz);
		}

		template <typename T>
		const Octant OctTreeNode<T>::calcOctantOfBounds(const Bounds& lowerBound, const Bounds& upperBound) const {
			//first octant (+,+,+)
			if (lowerBound.x >= x && lowerBound.y >= y && lowerBound.z >= z) return Octant::first;
			//second octant (-,+,+)
			if (upperBound.x < x && lowerBound.y >= y && lowerBound.z >= z) return Octant::second;
			//third octant (-,-,+)
			if (upperBound.x < x && upperBound.y < y && lowerBound.z >= z) return Octant::third;
			//fourth octant (+,-,+)
			if (lowerBound.x >= x && upperBound.y < y && lowerBound.z >= z) return Octant::fourth;
			//fifth octant (+,+,-)
			if (lowerBound.x >= x && lowerBound.y >= y && upperBound.z < z) return Octant::fifth;
			//sixth octant (-,+,-)
			if (upperBound.x < x && lowerBound.y >= y && upperBound.z < z) return Octant::sixth;
			//seventh octant (-,-,-)
			if (upperBound.x < x && upperBound.y < y && upperBound.z < z) return Octant::seventh;
			//eigth octant (+,-,-)
			if (lowerBound.x >= x && upperBound.y < y && upperBound.z < z) return Octant::eighth;
			return Octant::none;
		}

		template <typename T>
		const Octant OctTreeNode<T>::calcOctantOfPoint(const double& x, const double& y, const double& z) const {
			
			//first octant (+,+,+)
			if (x >= this->x && y >= this->y && z >= this->z) return Octant::first;
			//second octant (-,+,+)
			else if (x < this->x && y >= this->y && z >= this->z) return Octant::second;
			//third octant (-,-,+)
			else if (x < this->x && y < this->y && z >= this->z) return Octant::third;
			//fourth octant (+,-,+)
			else if (x >= this->x && y < this->y && z >= this->z) return Octant::fourth;
			//fifth octant (+,+,-)
			else if (x >= this->x && y >= this->y && z < this->z) return Octant::fifth;
			//sixth octant (-,+,-)
			else if (x < this->x && y >= this->y && z < this->z) return Octant::sixth;
			//seventh octant (-,-,-)
			else if (x < this->x && y < this->y && z < this->z) return Octant::seventh;
			//eigth octant (+,-,-)
			else return Octant::eighth;
		}

		template <typename T>
		const unordered_set<Octant> OctTreeNode<T>::findInvolvedOctants(const Bounds& lowerBound, const Bounds& upperBound) const {
			unordered_set<Octant> octantSet;
			//first octant (+,+,+)
			octantSet.insert(calcOctantOfPoint(upperBound.x, upperBound.y, upperBound.z));
			//second octant (-,+,+)
			octantSet.insert(calcOctantOfPoint(lowerBound.x, upperBound.y, upperBound.z));
			//third octant (-,-,+)
			octantSet.insert(calcOctantOfPoint(lowerBound.x, lowerBound.y, upperBound.z));
			//fourth octant (+,-,+)
			octantSet.insert(calcOctantOfPoint(upperBound.x, lowerBound.y, upperBound.z));
			//fifth octant (+,+,-)
			octantSet.insert(calcOctantOfPoint(upperBound.x, upperBound.y, lowerBound.z));
			//sixth octant (-,+,-)
			octantSet.insert(calcOctantOfPoint(lowerBound.x, upperBound.y, lowerBound.z));
			//seventh octant (-,-,-)
			octantSet.insert(calcOctantOfPoint(lowerBound.x, lowerBound.y, lowerBound.z));
			//eigth octant (+,-,-)
			octantSet.insert(calcOctantOfPoint(upperBound.x, lowerBound.y, lowerBound.z));
			return octantSet;
		}

		template <typename T>
		const double OctTreeNode<T>::calcMinDistance(const double& x, const double& y, const double& z) const {
			double xNearest = (std::fabs(lower.x - x) <= std::fabs(upper.x - x)) ? lower.x : upper.x;
			double yNearest = (std::fabs(lower.y - y) <= std::fabs(upper.y - y)) ? lower.y : upper.y;
			double zNearest = (std::fabs(lower.z - z) <= std::fabs(upper.z - z)) ? lower.z : upper.z;
			xNearest = xNearest - x;
			yNearest = yNearest - y;
			zNearest = zNearest - z;
			xNearest = (lower.x > x || x > upper.x) ? xNearest : 0.0;
			yNearest = (lower.y > y || y > upper.y) ? yNearest : 0.0;
			zNearest = (lower.z > z || z > upper.z) ? zNearest : 0.0;
			//std::cout << "min dist: " << std::sqrt(xNearest*xNearest + yNearest*yNearest + zNearest*zNearest) << " for node: \n" << toString() << std::endl;
			return std::sqrt(xNearest*xNearest + yNearest*yNearest + zNearest*zNearest);
		}

		template <typename T>
		void OctTreeNode<T>::addId(const T& id) {
			ids.insert(id);
			empty = false;
		}

		template <typename T>
		const IdSet<T>& OctTreeNode<T>::getIds() const {
			return ids;
		}

		template <typename T>
		const std::string OctTreeNode<T>::toString() const {
			std::stringstream ss;
			ss << "++++++++++++++++\n";
			ss << " Node is " << ((root) ? std::string("root") : std::string("not Root")) << " , is " << ((empty) ? std::string("empty") : std::string("not empty") )
				<< " and is " << ((leaf) ? std::string("leaf") : std::string("not leaf")) << ".\n";
			ss << " This Node contains " << ids.size() << " ids, which are:\n";
			for (auto id : ids) {
				ss << " ID: " << id << "\n";
			}
			ss << " Upper Bounds are: x=" << upper.x << ", y=" << upper.y << ", z=" << upper.z << "\n";
			ss << " Lower Bounds are: x=" << lower.x << ", y=" << lower.y << ", z=" << lower.z << "\n";
			ss << " Center lies in: x=" << x << ", y=" << y << ", z=" << z << "\n";
			ss << " Width is: " << width << ", Height is: " << height << ", Length is: " << length << "\n";
			ss << "++++++++++++++++\n";
			return ss.str();
		}
		
		template <typename T>
		OctTreeNode<T>::OctTreeNode() {
			parent = WeakNodePtr<T>();
			octants = Octants<T>(8);
			root = true;
			empty = true;
			leaf = true;
			lower = Bounds(0, 0, 0);
			upper = Bounds(0, 0, 0);
			ids = IdSet<T>();
			x = y = z = 0;
			width = height = length = 0;
		}
		
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
		void OctTree<T>::insertNode(const T& id, const Bounds& lowerBound, const Bounds& upperBound) {
			
			if (allowResize && !root->isInBounds(lowerBound, upperBound)) {
				//resize
				resize(lowerBound, upperBound);
			}
			std::cout << root->toString() << std::endl;
			
			insertNode(root, id, lowerBound, upperBound);
		}

		template <typename T>
		double OctTree<T>::getNearestDistance(const double& x, const double& y, const double& z) {
			NodePtr<T> node = getNearest(x, y, z);

			if (node->isRoot()) return POS_INF;
			return node->calcMinDistance(x, y, z);
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
			std::priority_queue<NodePtr<T>, std::vector<NodePtr<T>>, mycomparison> queue(mycomparison(x, y, z, true));
			queue.push(root);
			while (!queue.empty()) {
				NodePtr<T> current = queue.top();
				queue.pop();
				bool ignore = false;
				for (auto id : ignoreIDs) {
					if (current->getIds().find(id) != current->getIds().end()) {
						ignore = true;
						break;
					}
				}
				if (current->isLeaf() && !ignore) return current;
				for (unsigned int oct = Octant::first; oct <= Octant::eighth; oct++) {
					if (current->isOctantNull(static_cast<Octant>(oct))) continue;
					queue.push(current->getChild(static_cast<Octant>(oct)));
				}
			}
			return root;
		}

		template <typename T>
		bool OctTree<T>::insertNode(NodePtr<T> node, const T& id, const Bounds& lowerBound, const Bounds& upperBound) {
			Bounds diameter = node->getDiameter();
			// recursion anchor
			if ( (diameter.x <= minDiameter.x) || (diameter.y <= minDiameter.y) || (diameter.z <= minDiameter.z) ) {
				node->addId(id);
				//std::cout << node->toString() << std::endl;
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
					insertNode(node->getChild(oct), id, lbs[c], ubs[c]);
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
				std::cout << "Resize Step: " << steps << std::endl;
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
					std::cout << "New Octant: " << i << " : \n";
					std::cout << root->getChild(static_cast<Octant>(i))->toString() << std::endl;
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

		template class OctTreeNode<double>;
		template class OctTree<double>;
		template class OctTreeNode<unsigned int>;
		template class OctTree<unsigned int>;
	}
}