#pragma once

#include <string>
#include <memory>

namespace simobj {
	namespace shapes {

		using std::string;

		namespace types {
			enum ShapeType {
				Sphere, Cylinder, Ellipsoid
			};

			static const string TypeNames[] = { "Sphere", "Cylinder", "Ellipsoid" };
		}
		typedef types::ShapeType ShapeType;

		class BoundingBox {
		public:
			double x, y, z;
			double width, height, length;
			BoundingBox() {};
			BoundingBox(const double& width, const double& height, const double& length, const double& x=0.0, const double& y=0.0, const double& z=0.0);
			const string toString() const;
		};


		class Shape {
		protected:
			BoundingBox boundingBox;
			string typeName;
			ShapeType type;

			virtual void calcBoundingBox() = 0;
			Shape(const ShapeType& type);
			
		public:
			inline virtual ~Shape() {};
			inline const BoundingBox& getBoundingBox() const { return boundingBox; };
			const ShapeType& getType() const { return type; };
			const virtual string toString() const = 0;
		
		};

		class Sphere : public Shape {
		public:
			static Sphere* create(const double& radius);
			const virtual string toString() const;
			inline const double& getRadius() const { return radius; };
		protected:
			Sphere(const double& radius);
			double radius;
		private:
			virtual void calcBoundingBox();
		};

		class Cylinder : public Shape {
		public:
			static Cylinder* create(const double& radius, const double& length);
			const virtual string toString() const;
			const double& getRadius() const { return radius; };
			const double& getLength() const { return length; };
		protected:
			Cylinder(const double& radius, const double& length);
			double radius, length;
		private:
			virtual void calcBoundingBox();
		};

		class Ellipsoid : public Shape {
		public:
			static Ellipsoid* create(const double& rx, const double& ry, const double& rz);
			const virtual string toString() const;
			const double& getRadiusX() const { return rx; };
			const double& getRadiusY() const { return ry; };
			const double& getRadiusZ() const { return rz; };
		protected:
			Ellipsoid(const double& rx, const double& ry, const double& rz);
			double rx, ry, rz;
		private:
			virtual void calcBoundingBox();
		};

		using std::shared_ptr;
		using ShapePtr = shared_ptr<Shape>;

		class ShapeFactory {
		private:

			static ShapePtr createSphere(const double& radius);

			static ShapePtr createCylinder(const double& radius, const double& length);

			static ShapePtr createEllipsoid(const double& rx, const double& ry, const double& rz);

			static ShapePtr createInternal(const ShapeType& shapeType, const double& a, const double& b, const double& c);

			static ShapePtr createInternal(const ShapeType& shapeType, const double& a, const double& b);

			static ShapePtr createInternal(const ShapeType& shapeType, const double& a);

		public:
			template<typename... Args>
			static ShapePtr create(const ShapeType& shapeType, Args... args)
			{
				return createInternal(shapeType, args...);
			}
		};

	}
}