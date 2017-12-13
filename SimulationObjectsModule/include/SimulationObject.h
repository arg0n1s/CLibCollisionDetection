#pragma once

#include <iostream>
#include <memory>
#include <Eigen\Core>
#include <Eigen\Geometry>

namespace simobj {

	using std::string;
	using Eigen::Vector3d;
	using Quaternion = Eigen::Quaternion<double>;

	class SimulationObject {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			typedef std::shared_ptr<SimulationObject> SimObjPtr;

		virtual void method1() = 0;

		const string& getType() const;
		const unsigned long& getId() const;
		const Quaternion& getOrientation() const;
		const Vector3d& getPosition() const;

		void setOrientation(const Quaternion& orientation);
		void setPosition(const Vector3d& position);

	protected:
		string type;
		unsigned long id;
		Vector3d position;
		Quaternion orientation;

		SimulationObject(const unsigned long& id, const string& type);
		virtual ~SimulationObject();

	};
}