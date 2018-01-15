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

	const Vector3d AgentCluster::getPosition(const ReferenceFrame& frame) const {
		return position;
	}

	const Quaternion AgentCluster::getOrientation(const ReferenceFrame& frame) const {
		return orientation;
	}

	SimObjPtr AgentCluster::createInternal(const unsigned long& id, const string& type) {
		return SimObjPtr(new AgentCluster(id, type));
	}

	void AgentCluster::rotateCluster(const Quaternion& rotation) {
		orientation = orientation*rotation;
	}
	void AgentCluster::moveCluster(const Vector3d& translation) {
		position = position + translation;
	}

	void AgentCluster::insertAgent(SimObjPtr agent) {
		if (isAgentInCluster(agent->getId())) throw std::runtime_error("Agent with given ID already exist within this cluster.");
		//shared_ptr<Agent> agnt = std::static_pointer_cast<Agent>(agent);
		//agnt->setAgentCluster(SimObjPtr(this));
		agents.insert(std::make_pair(agent->getId(), std::static_pointer_cast<Agent>(agent)));
	}

	SimObjPtr AgentCluster::getAgent(const unsigned long& id) {
		if (!isAgentInCluster(id)) throw std::runtime_error("Agent with given ID does not exist within this cluster.");
		return agents[id];
	}

	const unordered_map& AgentCluster::getAllAgents() const {
		return agents;
	}

	const bool AgentCluster::isAgentInCluster(const unsigned long& id) const {
		return agents.find(id) != agents.end();
	}
}