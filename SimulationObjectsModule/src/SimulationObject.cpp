#include "..\include\SimulationObject.h"


SimulationObject::SimulationObject() {
	std::cout << "Creating SimulationObject" << std::endl;
	type = "";
	id = "";
	orientation = Quaternion::Identity();
	position = Vector3d::Zero();
}
SimulationObject::~SimulationObject() {
	std::cout << "Destroying SimulationObject" << std::endl;
}

const std::string& SimulationObject::getType() const {
	return type;
}

const std::string& SimulationObject::getId() const {
	return id;
}

const Quaternion& SimulationObject::getOrientation() const {
	return orientation;
}

const Vector3d& SimulationObject::getPosition() const {
	return position;
}