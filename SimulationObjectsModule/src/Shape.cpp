#include "../include/Shape.h"

namespace simobj {
	namespace shapes {

		Sphere::Sphere(const double& radius) : radius(radius) {
			calcBoundingBox();
		}

		void Sphere::calcBoundingBox() {
			boundingBox = BoundingBox();
			boundingBox.x = 0; boundingBox.y = 0; boundingBox.z = 0;
			boundingBox.width = radius*2; boundingBox.height = radius * 2; boundingBox.length = radius * 2;
		}

		Cylinder::Cylinder(const double& radius, const double& length) : radius(radius), length(length) {
			calcBoundingBox();
		}

		void Cylinder::calcBoundingBox() {
			boundingBox = BoundingBox();
			boundingBox.x = 0; boundingBox.y = 0; boundingBox.z = 0;
			boundingBox.width = radius * 2; boundingBox.height = radius * 2; boundingBox.length = length;
		}

		Ellipsoid::Ellipsoid(const double& rx, const double& ry, const double& rz) : rx(rx), ry(ry), rz(rz) {
			calcBoundingBox();
		}

		void Ellipsoid::calcBoundingBox() {
			boundingBox = BoundingBox();
			boundingBox.x = 0; boundingBox.y = 0; boundingBox.z = 0;
			boundingBox.width = rx * 2; boundingBox.height = ry * 2; boundingBox.length = rz*2;
		}

	}
}
