#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"
#include <unordered_map>

namespace simobj {

	namespace shapes {
		class Shape;
	}

	using std::string;
	using shapes::Shape;
	using ShapePtr = std::shared_ptr<Shape>;
	using SimObjPtr = SimulationObject::SimObjPtr;
	using SitesMap = std::unordered_map<unsigned long, SimObjPtr>;

	class Agent :
		public SimulationObject, public SimulationObjectFactory<Agent, SimulationObject>
	{
	public:
		~Agent();

		virtual void method1() {};
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void addSite(SimObjPtr site);

		void setAgentCluster(SimObjPtr cluster);
		void setShape(ShapePtr shape);

		ShapePtr getShape();
		SimObjPtr getAgentCluster();

		bool isInAnyCluster() const;
		bool isAgentCluster(SimObjPtr cluster) const;
	protected:
		Agent(const unsigned long& id, const string& type);
		SitesMap sites;
		ShapePtr shape;
		SimObjPtr cluster;
		bool belongsToCluster;
	};
}