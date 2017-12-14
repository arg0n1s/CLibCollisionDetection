#include "../include/Shape.h"
#include <sstream>

namespace simobj {
	namespace shapes {

		Shape::Shape(const ShapeType& type) : type(type) { 
			boundingBox = BoundingBox(); 
			typeName = types::TypeNames[type];
		};

		BoundingBox::BoundingBox(const double& width, const double& height, const double& length, const double& x, const double& y, const double& z) :
			width(width), height(height), length(length), x(x), y(y), z(z) {}

		const string BoundingBox::toString() const {
			std::stringstream ss;
			ss << "BBox: [ x: " << x << ", y: " << y << ", z: " << z << "\n";
			ss << "Width: " << width << ", Height: " << height << ", Length: " << length << " ]\n";
			return ss.str();
		}

		Sphere::Sphere(const double& radius) : Shape(ShapeType::Sphere), radius(radius) {
			calcBoundingBox();
		}

		void Sphere::calcBoundingBox() {
			boundingBox = BoundingBox(radius * 2, radius * 2, radius * 2);
		}

		Sphere* Sphere::create(const double& radius) {
			return new Sphere(radius);
		}

		const string Sphere::toString() const {
			std::stringstream ss;
			ss << "Shape [ Type: "<<typeName<< ", \n";
			ss << "\t Radius: " << radius << ", \n";
			ss << "\t BoundingBox: \n";
			ss << boundingBox.toString();
			ss << "] \n";
			return ss.str();
		}

		Cylinder::Cylinder(const double& radius, const double& length) : Shape(ShapeType::Cylinder), radius(radius), length(length) {
			calcBoundingBox();
		}

		void Cylinder::calcBoundingBox() {
			boundingBox = BoundingBox(radius * 2, radius * 2, length);
		}

		Cylinder* Cylinder::create(const double& radius, const double& length) {
			return new Cylinder(radius, length);
		}

		const string Cylinder::toString() const {
			std::stringstream ss;
			ss << "Shape [ Type: " << typeName << ", \n";
			ss << "\t Length: "<< length << ", Radius: " << radius << ", \n";
			ss << "\t BoundingBox: \n";
			ss << boundingBox.toString();
			ss << "] \n";
			return ss.str();
		}

		Ellipsoid::Ellipsoid(const double& rx, const double& ry, const double& rz) : Shape(ShapeType::Ellipsoid), rx(rx), ry(ry), rz(rz) {
			calcBoundingBox();
		}

		void Ellipsoid::calcBoundingBox() {
			boundingBox = BoundingBox(rx * 2, ry * 2, rz * 2);
		}

		Ellipsoid* Ellipsoid::create(const double& rx, const double& ry, const double& rz) {
			return new Ellipsoid(rx, ry, rz);
		}

		const string Ellipsoid::toString() const {
			std::stringstream ss;
			ss << "Shape [ Type: " << typeName << ", \n";
			ss << "\t Radius along x(a): " << rx << ", Radius along y(b): " << ry << ", Radius along z(c): " << rz << ", \n";
			ss << "\t BoundingBox: \n";
			ss << boundingBox.toString();
			ss << "] \n";
			return ss.str();
		}

		ShapePtr ShapeFactory::createSphere(const double& radius)
		{
			return shared_ptr<Shape>(Sphere::create(radius));
		}

		ShapePtr ShapeFactory::createCylinder(const double& radius, const double& length)
		{
			return shared_ptr<Shape>(Cylinder::create(radius, length));
		}

		ShapePtr ShapeFactory::createEllipsoid(const double& rx, const double& ry, const double& rz)
		{
			return shared_ptr<Shape>(Ellipsoid::create(rx, ry, rz));
		}

		ShapePtr ShapeFactory::create(const ShapeType& shapeType, const double& a, const double& b, const double& c) {
			switch (shapeType) {
			case ShapeType::Ellipsoid: return createEllipsoid(a, b, c);
				break;
			default: return nullptr;
			}
		}

		ShapePtr ShapeFactory::create(const ShapeType& shapeType, const double& a, const double& b) {
			switch (shapeType) {
			case ShapeType::Cylinder: return createCylinder(a, b);
				break;
			default: return nullptr;
			}
		}

		ShapePtr ShapeFactory::create(const ShapeType& shapeType, const double& a) {
			switch (shapeType) {
			case ShapeType::Sphere: return createSphere(a);
				break;
			default: return nullptr;
			}
		}

	}
}
