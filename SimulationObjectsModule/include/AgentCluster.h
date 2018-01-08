#pragma once
#include "SimulationObject.h"
#include "SimulationObjectFactory.h"
#include <unordered_map>

namespace simobj {

	using SimObjPtr = SimulationObject::SimObjPtr;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;


	class AgentCluster :
		public SimulationObject, public SimulationObjectFactory<AgentCluster, SimulationObject>
	{
	public:
		~AgentCluster();

		virtual string toString() const;
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const;
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const;
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void insertAgent(SimObjPtr agent);
		SimObjPtr getAgent(const unsigned long& id);
		const unordered_map& getAllAgents() const;
		const bool isAgentInCluster(const unsigned long& id) const;

	protected:
		AgentCluster(const unsigned long& id, const string& type);
		unordered_map agents;
	};
}