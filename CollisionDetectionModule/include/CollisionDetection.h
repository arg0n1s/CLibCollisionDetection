#pragma once
#include <memory>
#include <unordered_map>

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

	class CollisionDetection {
	public:
		CollisionDetection();
		void setInitialTreeDiameter(const double& initialTreeDiameter);
		void setMinimalCellDiameter(const double& minimalCellDiameter);
		void setAllowRescaling(const bool rescalingOn);

		void makeTreeFromCluster(SimObjPtr cluster);
		TreePtr getTree(const unsigned int& id);
	private:
		TreeMap trees;
		double initialTreeDiameter;
		double minimalCellDiameter;
		bool allowRescaling;
	};
}