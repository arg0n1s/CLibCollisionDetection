#pragma once
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace simobj {
	class SimulationObject;
	using SimObjPtr = std::shared_ptr<SimulationObject>;
}

namespace collision {
	namespace octtree {
		template <typename T>
		class OctTree;

	}

	using std::shared_ptr;
	using octtree::OctTree;
	using TreePtr = shared_ptr<OctTree<unsigned int>>;
	using TreeMap = std::unordered_map<unsigned int, TreePtr>;
	using simobj::SimulationObject;
	using simobj::SimObjPtr;
	using IDSet = std::unordered_set<unsigned int>;

	class CollisionDetection {
	public:
		CollisionDetection();
		void setInitialTreeDiameter(const double& initialTreeDiameter);
		void setMinimalCellDiameter(const double& minimalCellDiameter);
		void setAllowRescaling(const bool rescalingOn);

		const bool isClusterInTree(const unsigned int& id) const;
		void makeTreeFromCluster(SimObjPtr cluster);
		void addAgentToTree(SimObjPtr agent);
		TreePtr getTree(const unsigned int& id);
		const bool checkForCollision(SimObjPtr cluster, const IDSet& ignoreIDs, SimObjPtr candidate, SimObjPtr& nearest, double& nearestDistance);
		const double calcBodyToBodyDistance(SimObjPtr body1, SimObjPtr body2) const;
	private:
		TreeMap trees;
		double initialTreeDiameter;
		double minimalCellDiameter;
		bool allowRescaling;

		const double calcSphereToSphereDistance(SimObjPtr sphere1, SimObjPtr sphere2) const;
		const double calcSphereToCylinderDistance(SimObjPtr sphere, SimObjPtr cylinder) const;
		const double calcCylinderToCylinderDistance(SimObjPtr cylinder1, SimObjPtr cylinder2) const;
	};
}