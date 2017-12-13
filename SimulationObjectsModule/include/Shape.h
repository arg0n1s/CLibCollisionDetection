#pragma once

#include <string>
#include <sstream>

namespace simobj {
	namespace shapes {

		typedef struct {
			double x, y, z;
			double width, height, length;

			inline const std::string toString() const {
				std::stringstream ss;
				ss << "Origin: [" << x << ", " << y << ", " << z << "]\n";
				ss << "BBox: [ Width: " << width << ", Height: " << height << ", Length: " << length << " ]\n";
				return ss.str();
			}
		}BoundingBox;


		class Shape {
		protected:
			virtual void calcBoundingBox() = 0;
			inline Shape() {};
			BoundingBox boundingBox;

		public:
			inline virtual ~Shape() {};
			inline const BoundingBox& getBoundingBox() const { return boundingBox; };
		
		};

		class Sphere : public Shape {
		public:
			Sphere(const double& radius);
		protected:
			double radius;
		private:
			virtual void calcBoundingBox();
		};

		class Cylinder : public Shape {
		public:
			Cylinder(const double& radius, const double& length);
		protected:
			double radius, length;
		private:
			virtual void calcBoundingBox();
		};

		class Ellipsoid : public Shape {
		public:
			Ellipsoid(const double& rx, const double& ry, const double& rz);
		protected:
			double rx, ry, rz;
		private:
			virtual void calcBoundingBox();
		};

	}
}