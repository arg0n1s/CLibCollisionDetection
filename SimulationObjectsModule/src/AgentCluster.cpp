#include "..\include\AgentCluster.h"
#include "..\include\Agent.h"
#include <sstream>

namespace simobj {
	using SimObjPtr = SimulationObject::SimObjPtr;
	using std::string;
	using unordered_map = std::unordered_map<unsigned long, SimObjPtr>;

	AgentCluster::AgentCluster(const unsigned long& id, const string& type) : SimulationObject(id, type)
	{
		agents = unordered_map();
	}


	AgentCluster::~AgentCluster()
	{
	}

	string AgentCluster::toString() const {
		std::stringstream ss;
		ss << "Agent Cluster: [" << " Type: " << type << ", ID: " << id << ",\n";
		ss << "\t x: " << position.x() << ", y: " << position.y() << ", z: " << position.z() << ", \n";
		ss << "\t qw: " << orientation.w() << ", qx: " << orientation.x() << ", qy: " << orientation.y() << ", qz: " << orientation.z() << ", \n";

		ss << "\t attached agents: \n";
		for (auto agent : agents) {
			ss << agent.second->toString();
		}
		ss << "] \n";
		return ss.str();
	}

	SimObjPtr AgentCluster::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new AgentCluster(id, type));
	}

	void AgentCluster::insertAgent(SimObjPtr agent) {
		agents.insert(std::make_pair(agent->getId(), std::static_pointer_cast<Agent>(agent)));
	}

	SimObjPtr AgentCluster::getAgent(const unsigned long& id) {
		return agents[id];
	}

	const unordered_map& AgentCluster::getAllAgents() const {
		return agents;
	}

	Vector3d AgentCluster::getConvertedPosition(const Vector3d& position) const {
		return orientation*position + this->position;
	}

	Quaternion AgentCluster::getConvertedOrientation(const Quaternion& orientation) const {
		return orientation*this->orientation;
	}

	bool AgentCluster::isAgentInCluster(const unsigned long& id) {
		return agents.find(id) != agents.end();
	}
}