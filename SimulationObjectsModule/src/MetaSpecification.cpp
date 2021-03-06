#include "../include/MetaSpecification.h"
#include "../include/Shape.h"

namespace simobj {

	namespace specs {

		MetaSpecification::MetaSpecification() {
			agentSpecs = AgentSpecsMap();
		}
		void MetaSpecification::addAgentSpecification(const AgentSpecification& agentSpec) {
			if (isAgentInSpecs(agentSpec.getType())) throw std::runtime_error("AgentSpec with given type already present withing Map of AgentSpecs!");
			agentSpecs.insert(std::make_pair(agentSpec.getType(), agentSpec));
		}

		const AgentSpecification& MetaSpecification::getAgentSpecification(const string& type) const {
			if (!isAgentInSpecs(type)) throw std::runtime_error("AgentSpec with given type not present withing Map of AgentSpecs!");
			return agentSpecs.at(type);
		}

		const bool MetaSpecification::isAgentInSpecs(const string& type) const {
			return agentSpecs.find(type) != agentSpecs.end();
		}

		AgentSpecification::AgentSpecification(const string& type, ShapePtr shape) : type(type), shape(shape) {
			siteSpecs = SiteSpecsMap();
		}

		const SiteSpecsArray AgentSpecification::getSiteSpecifications() const {
			SiteSpecsArray out;
			for (auto siteSpec : siteSpecs) {
				out.push_back(siteSpec.second);
			}
			return out;
		}

		const bool AgentSpecification::isSiteInSpecs(const unsigned int& id) const {
			return siteSpecs.find(id) != siteSpecs.end();
		}

		const string& AgentSpecification::getType() const {
			return type;
		}

		const ShapePtr AgentSpecification::getShape() const {
			return shape;
		}

		void AgentSpecification::addSiteSpecification(const SiteSpecification& siteSpec) {
			if (isSiteInSpecs(siteSpec.getId())) throw std::runtime_error("SiteSpec with given id already present withing Map of SiteSpecs!");
			siteSpecs.insert(std::make_pair(siteSpec.getId(), siteSpec));
		}

		SiteSpecification::SiteSpecification(const unsigned long& id, const string& type, const Vector3d& coordinates, const CoordinateType& cType) : 
			id(id), type(type), coordinates(coordinates), cType(cType) {};

		const unsigned long& SiteSpecification::getId() const {
			return id;
		}
		const string& SiteSpecification::getType() const {
			return type;
		}
		const Vector3d& SiteSpecification::getCoordinates() const {
			return coordinates;
		}

		const CoordinateType& SiteSpecification::getCoordinateType() const {
			return cType;
		}

	}
}