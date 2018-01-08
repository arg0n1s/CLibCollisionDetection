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

		virtual string toString() const;
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void addSite(SimObjPtr site);

		void setAgentCluster(SimObjPtr cluster);
		void setShape(ShapePtr shape);

		SimObjPtr getSite(const unsigned long& id);
		const SitesMap& getAllSites() const;
		ShapePtr getShape();
		SimObjPtr getAgentCluster();

		void rotateAgent(const Quaternion& rotation);

		bool isInAnyCluster() const;
		bool isAgentCluster(SimObjPtr cluster) const;
	protected:
		Agent(const unsigned long& id, const string& type);
		SitesMap sites;
		ShapePtr shape;
		SimObjPtr cluster;
		bool belongsToCluster;
		bool hasShape;
	};
}