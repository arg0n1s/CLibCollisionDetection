#include "../include/MetaSpecification.h"
#include "../include/Shape.h"

namespace simobj {

	namespace specs {

		MetaSpecification::MetaSpecification() {
			agentSpecs = AgentSpecsMap();
		}
		void MetaSpecification::addAgentSpecification(const AgentSpecification& agentSpec) {
			agentSpecs.insert(std::make_pair(agentSpec.getType(), agentSpec));
		}

		const AgentSpecification& MetaSpecification::getAgentSpecification(const string& type) const {
			return agentSpecs.at(type);
		}

		const bool MetaSpecification::isAgentInSpecs(const string& type) const {
			return agentSpecs.find(type) != agentSpecs.end();
		}

		AgentSpecification::AgentSpecification(const string& type, ShapePtr shape) : type(type), shape(shape) {
			siteSpecs = SiteSpecsArray();
		}

		const SiteSpecsArray& AgentSpecification::getSiteSpecifications() const {
			return siteSpecs;
		}

		const string& AgentSpecification::getType() const {
			return type;
		}

		const ShapePtr AgentSpecification::getShape() const {
			return shape;
		}

		void AgentSpecification::addSiteSpecification(const SiteSpecification& siteSpec) {
			siteSpecs.push_back(siteSpec);
		}

		SiteSpecification::SiteSpecification(const unsigned long& id, const string& type, const Vector3d& position) : id(id), type(type), position(position) {};

		const unsigned long& SiteSpecification::getId() const {
			return id;
		}
		const string& SiteSpecification::getType() const {
			return type;
		}
		const Vector3d& SiteSpecification::getPosition() const {
			return position;
		}

	}
}