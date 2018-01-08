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

		Vector3d Sphere::karthesianToParametrizedCoordinates(const Vector3d& karth) {
			double r, theta, phi;
			r = std::sqrt(karth.x()*karth.x() + karth.y()*karth.y() + karth.z()*karth.z());
			if (r = 0.0) throw std::runtime_error("Division by zero error while converting from karth. to spherical coordinates!");
			theta = std::acos(karth.z() / r);
			phi = std::atan2(karth.y(), karth.x());
			return Vector3d(r, theta, phi);
		}

		Vector3d Sphere::parametrizedToKarthesianCoordinates(const Vector3d& param) {
			double x, y, z;
			x = param.x() * std::sin(param.y()) * std::cos(param.z());
			y = param.x() * std::sin(param.y()) * std::sin(param.z());
			z = param.x() * std::cos(param.y());
			return Vector3d(x, y, z);
		}

		Vector3d Sphere::hullIntersectionFromKarthPointer(const Vector3d& karthPointer) {
			Vector3d param = karthesianToParametrizedCoordinates(karthPointer);
			param.x() = radius;
			return parametrizedToKarthesianCoordinates(param);
		}

		Vector3d Sphere::hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer) {
			Vector3d param = paramPointer;
			param.x() = radius;
			return parametrizedToKarthesianCoordinates(param);
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

		Vector3d Cylinder::karthesianToParametrizedCoordinates(const Vector3d& karth) {
			double r, phi, z;
			r = std::sqrt(karth.x()*karth.x() + karth.y()*karth.y());
			if (r = 0.0) throw std::runtime_error("Division by zero error while converting from karth. to cylindrical coordinates!");
			phi = std::atan2(karth.y(), karth.x());
			z = karth.z();
			return Vector3d(r, phi, z);
		}

		Vector3d Cylinder::parametrizedToKarthesianCoordinates(const Vector3d& param) {
			double x, y, z;
			x = param.x() * std::cos(param.y());
			y = param.x() * std::sin(param.y());
			z = param.z();
			return Vector3d(x, y, z);
		}

		Vector3d Cylinder::hullIntersectionFromKarthPointer(const Vector3d& karthPointer) {
			Vector3d param = karthesianToParametrizedCoordinates(karthPointer);
			param.x() = radius;
			return parametrizedToKarthesianCoordinates(param);
		}

		Vector3d Cylinder::hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer) {
			Vector3d param = paramPointer;
			param.x() = radius;
			return parametrizedToKarthesianCoordinates(param);
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

		Vector3d Ellipsoid::karthesianToParametrizedCoordinates(const Vector3d& karth) {
			double theta, phi;
			theta = std::acos(karth.z() / 1.0);
			phi = std::atan2(karth.y(), karth.x());
			return Vector3d(1, theta, phi);
		}

		Vector3d Ellipsoid::parametrizedToKarthesianCoordinates(const Vector3d& param) {
			double x, y, z;
			x = rx * std::sin(param.y()) * std::cos(param.z());
			y = ry * std::sin(param.y()) * std::sin(param.z());
			z = rz * std::cos(param.y());
			return Vector3d(x, y, z);
		}

		Vector3d Ellipsoid::hullIntersectionFromKarthPointer(const Vector3d& karthPointer) {
			Vector3d param = karthesianToParametrizedCoordinates(karthPointer);
			return parametrizedToKarthesianCoordinates(param);
		}

		Vector3d Ellipsoid::hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer) {
			return parametrizedToKarthesianCoordinates(paramPointer);
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

		ShapePtr ShapeFactory::createInternal(const ShapeType& shapeType, const double& a, const double& b, const double& c) {
			switch (shapeType) {
			case ShapeType::Ellipsoid: return createEllipsoid(a, b, c);
				break;
			default: return nullptr;
			}
		}

		ShapePtr ShapeFactory::createInternal(const ShapeType& shapeType, const double& a, const double& b) {
			switch (shapeType) {
			case ShapeType::Cylinder: return createCylinder(a, b);
				break;
			default: return nullptr;
			}
		}

		ShapePtr ShapeFactory::createInternal(const ShapeType& shapeType, const double& a) {
			switch (shapeType) {
			case ShapeType::Sphere: return createSphere(a);
				break;
			default: return nullptr;
			}
		}

	}
}
