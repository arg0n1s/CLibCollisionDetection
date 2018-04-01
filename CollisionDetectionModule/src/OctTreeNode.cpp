#include "..\include\OctTreeNode.h"
#include <sstream>

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
			/*
			if (width < std::fabs(upperBound.x - lowerBound.x)) return false;
			if (height < std::fabs(upperBound.y - lowerBound.y)) return false;
			if (length < std::fabs(upperBound.z - lowerBound.z)) return false;
			*/
			if (!isInBounds(lowerBound, upperBound)) return false;
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
			return std::sqrt(xNearest*xNearest + yNearest * yNearest + zNearest * zNearest);
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
			ss << " Node is " << ((root) ? std::string("root") : std::string("not Root")) << " , is " << ((empty) ? std::string("empty") : std::string("not empty"))
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

		template class OctTreeNode<unsigned int>;
		template class OctTreeNode<double>;
	}
}