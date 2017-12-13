#pragma once
#include <memory>
#include <string>
namespace simobj {
	namespace specs {
		class AgentSpecification;
		class SiteSpecification;
	}
	using std::string;
	using specs::AgentSpecification;
	using specs::SiteSpecification;
	using std::shared_ptr;

	template <typename TClass, typename TInterface>
	class SimulationObjectFactory {
	public:
		static shared_ptr<TInterface> create(const unsigned long& id, const string& type);
		static shared_ptr<TInterface> create(const unsigned long& id, const AgentSpecification& agentSpec);
		
	};
}