#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <memory>

namespace simobj {
	
	namespace shapes {
		class Shape;
	}
	
	namespace specs {

		class AgentSpecification;
		class SiteSpecification;

		using std::string;
		using AgentSpecsMap = std::unordered_map<string, AgentSpecification>;
		using SiteSpecsArray = std::vector<SiteSpecification>;
		using Eigen::Vector3d;
		using simobj::shapes::Shape;
		using ShapePtr = std::shared_ptr<Shape>;

		class MetaSpecification {
		public:
			MetaSpecification();

			void addAgentSpecification(const AgentSpecification& agentSpec);
			const AgentSpecification& getAgentSpecification(const string& type) const;
			const bool isAgentInSpecs(const string& type) const;

		private:
			AgentSpecsMap agentSpecs;
		};

		class AgentSpecification {
		public:
			AgentSpecification(const string& type, ShapePtr shape);

			const string& getType() const;
			const ShapePtr getShape() const;
			void addSiteSpecification(const SiteSpecification& siteSpec);
			const SiteSpecsArray& getSiteSpecifications() const;
		private:
			string type;
			SiteSpecsArray siteSpecs;
			ShapePtr shape;
		};

		class SiteSpecification {
		public:
			SiteSpecification(const unsigned long& id, const string& type, const Vector3d& position);

			const unsigned long& getId() const;
			const string& getType() const;
			const Vector3d& getPosition() const;
		private:
			unsigned long id;
			string type;
			Vector3d position;
		};
	}
}