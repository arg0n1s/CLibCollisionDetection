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

		virtual void method1() {};
		static SimObjPtr createInternal(const unsigned long& id, const string& type);

		void insertAgent(SimObjPtr agent);
		SimObjPtr getAgent(const unsigned long& id);
		bool isAgentInCluster(const unsigned long& id);

	protected:
		AgentCluster(const unsigned long& id, const string& type);
		unordered_map agents;
	};
}