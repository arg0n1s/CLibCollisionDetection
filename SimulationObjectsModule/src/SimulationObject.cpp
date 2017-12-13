#include "..\include\SimulationObject.h"

namespace simobj {
	SimulationObject::SimulationObject(const unsigned long& id, const string& type) : type(type), id(id) {
		orientation = Quaternion::Identity();
		position = Vector3d::Zero();
	}
	SimulationObject::~SimulationObject() {
	}

	const std::string& SimulationObject::getType() const {
		return type;
	}

	const unsigned long& SimulationObject::getId() const {
		return id;
	}

	const Quaternion& SimulationObject::getOrientation() const {
		return orientation;
	}

	const Vector3d& SimulationObject::getPosition() const {
		return position;
	}

	void SimulationObject::setOrientation(const Quaternion& orientation) {
		this->orientation = orientation;
	}

	void SimulationObject::setPosition(const Vector3d& position) {
		this->position = position;
	}
}