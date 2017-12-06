#pragma once

#include <iostream>
#include <memory>
#include <Eigen\Core>
#include <Eigen\Geometry>

using std::string;
using Eigen::Vector3d;
using Quaternion = Eigen::Quaternion<double>;

class SimulationObject {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<SimulationObject> SimObjPtr;

	virtual void method1() = 0;

	const string& getType() const;
	const string& getId() const;
	const Quaternion& getOrientation() const;
	const Vector3d& getPosition() const;

protected:
	string type;
	string id;
	Vector3d position;
	Quaternion orientation;

	SimulationObject();
	virtual ~SimulationObject();
	
};