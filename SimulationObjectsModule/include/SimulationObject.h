#pragma once

#include <iostream>
#include <memory>
#include <Eigen\Core>
#include <Eigen\Geometry>

namespace simobj {

	using std::string;
	using Eigen::Vector3d;
	using Quaternion = Eigen::Quaternion<double>;

	namespace frames {
		enum ReferenceFrame {
			Local, Global
		};

	}
	typedef frames::ReferenceFrame ReferenceFrame;

	class SimulationObject {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		typedef std::shared_ptr<SimulationObject> SimObjPtr;
		typedef std::shared_ptr<const SimulationObject> SimObjConstPtr;
		typedef std::weak_ptr<SimulationObject> SimObjWeakPtr;
		typedef std::unique_ptr<SimulationObject> SimObjUniquePtr;

		virtual string toString() const = 0;
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const = 0;
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const = 0;

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