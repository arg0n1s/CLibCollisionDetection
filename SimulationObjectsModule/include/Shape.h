#pragma once

#include <string>
#include <memory>
#include <Eigen\Core>
#include <Eigen\Dense>

namespace simobj {
	namespace shapes {

		// Aliases for used namespaces
		using std::string;
		using Eigen::Vector3d;

		namespace types {

			/**
			\brief Enumerator used to create shape objects of a certain type
			in the shape object factory.
			*/
			enum ShapeType {
				Sphere, Cylinder, Ellipsoid
			};

			// Collection of string type names, used for toString methods and debugging purposes.
			static const string TypeNames[] = { "Sphere", "Cylinder", "Ellipsoid" };
		}

		// Typedef for ease of use
		typedef types::ShapeType ShapeType;

		/**
			\brief Helper class to store real vector bounds of a shape with a particular type.
			Bounds are defined by a Box with width (x-Axis), height (y-Axis), length (z-Axis)
			and center x,y,z.
			Bounds are used to insert shapes into a collision tree (e.g. OctTree, KdTree, etc..)
		*/
		class BoundingBox {
		public:

			/*Center of the bounding box.*/
			double x, y, z;

			/*Dimension of the bounding box.*/
			double width, height, length;

			/**
				\brief Default-Constructor
			*/
			BoundingBox() {};

			/**
				\brief Construct a new box given its center coordinates and spatial dimensions.
			*/
			BoundingBox(const double& width, const double& height, const double& length, const double& x=0.0, const double& y=0.0, const double& z=0.0);

			/**
				\brief Packs the bounding boxes internal information into a formatted readable string.
			*/
			const string toString() const;
		};

		/**
			\brief Abstract class that must be implemented by classes intended to serve as
			a shape for agents in the simulation.
			Shapes provide Agent-objects with information about their spatial characteristics,
			i.e. their hull, shape of their hull and their geometric origin.
			A shape inheriting this abstract class must implement procedures to:
			1) calculate a bounding box surrounding the complex shape (e.g. sphere in a box)
			2) calculate points on the complex shape's hull given a multitude of different parametrized coordinates
			3) represent the shape's internal information as string
			\note A shape is a necessity to perform collision checking between agents.
		*/
		class Shape {
		protected:

			/*Bounding box object of this shape.*/
			BoundingBox boundingBox;

			/*String representation of this shape's type.*/
			string typeName;

			/*This shape's type enumerator.*/
			ShapeType type;

			/**
				\brief Calculates the bounding box of this shape and stores it internally.
			 */
			virtual void calcBoundingBox() = 0;

			/**
				\brief Constructor
				\param[in] type Enumerator that defines this shape.
			*/
			Shape(const ShapeType& type);
			
		public:

			/**
				\brief Destructor
			*/
			inline virtual ~Shape() {};

			/**
				\brief Calculate the parametrized coordinates of this shape from
				given karthesian coordinates. Parametrization heavyly depends on the type of shape,
				e.g. sphere, cylinder, ellipsoid, etc.
				\example (0, 1, 0) in karth. coordinates correspond to (1, 0, pi/2 ) in spherical coordinates.
				\param[in] karth Karthesian coordinates that should be transformed to parametrized coordinates.
				\returns Parametrized coordinates from given karthesian coordinates.
			*/
			virtual Vector3d karthesianToParametrizedCoordinates(const Vector3d& karth) = 0;

			/**
				\brief Calculate karthesian coordinates from given parametrized coordinates of this shape. 
				Parametrization heavyly depends on the type of shape, e.g. sphere, cylinder, ellipsoid, etc.
				\example (1, 0, pi/2 ) in spheric coordinates correspond to (0, 1, 0) in karth. coordinates.
				\param[in] param Parametrized coordinates that should be transformed to karthesian coordinates.
				\returns Karthesian coordinates from given parametrized coordinates.
			*/
			virtual Vector3d parametrizedToKarthesianCoordinates(const Vector3d& param) = 0;

			/**
				\brief Calculates the coordinates of a point on this shape's hull, given a 3x1 vector
				in karth. coordinates pointing from the shape's origin to the point on the hull.
				\example (1, 1, 0) in karth. coordinates points to (1, pi/2, pi/2 ) on a spheres hull with radius 1.
				\param[in] karthPointer Karthesian 3x1 vector pointing to the shapes hull.
				\returns Karthesian coordinates of the resulting point on the hull.
			*/
			virtual Vector3d hullIntersectionFromKarthPointer(const Vector3d& karthPointer) = 0;

			/**
				\brief Calculates the coordinates of a point on this shape's hull, given the corresponding
				parametrized coordinates that intersect with the hull.
				\example (5, pi/2, pi/2) in spheric coordinates points to (1, pi/2, pi/2 ) on a sphere's hull with radius 1.
				\param[in] paramPointer Parametrized coordinates pointing to the shapes hull.
				\returns Karthesian coordinates of the resulting point on the hull.
			*/
			virtual Vector3d hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer) = 0;

			/**
				\brief Get this shape's bounding box.
				\returns This shape's bounding box object.
			*/
			inline const BoundingBox& getBoundingBox() const { return boundingBox; };

			/**
				\brief Get this shape's type.
				\returns This shape's type enumerator.
			*/
			inline const ShapeType& getType() const { return type; };

			/**
				\brief Packs the shape's internal information into a formatted readable string.
			*/
			const virtual string toString() const = 0;
		
		};

		/**
			\brief A class that implements the shape interface to represent spheres of any size.
		*/
		class Sphere : public Shape {
		public:
			/**
				\brief Call to instantiate a new Sphere-object.
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" a Sphere-object, i.e.Constructor is private!
				\param[in] radius Radius of the sphere.
			*/
			static Sphere* create(const double& radius);
			
			/**
				\brief Calculate the parametrized coordinates of a sphere from given karthesian coordinates.
				\param[in] karth Karthesian coordinates that should be transformed to spheric coordinates.
				\returns Spheric coordinates from given karthesian coordinates.
				\note Order of spheric coordinates in the returned 3x1 vector are: radius, theta, phi.
			*/
			virtual Vector3d karthesianToParametrizedCoordinates(const Vector3d& karth);
			
			/**
				\brief Calculate the karthesian coordinates from given parametrized coordinates of a sphere.
				\param[in] param Spheric coordinates that should be transformed to karthesian coordinates.
				\returns Karthesian coordinates from given Spheric coordinates.
				\note Order of spheric coordinates in the given 3x1 vector are: radius, theta, phi.
			*/
			virtual Vector3d parametrizedToKarthesianCoordinates(const Vector3d& param);

			/**
				\brief Calculates the coordinates of a point on this sphere's hull, given a 3x1 vector
				in karth. coordinates pointing from the spheres's origin to the point on the hull.
				\param[in] karthPointer Karthesian 3x1 vector pointing to the sphere's hull.
				\returns Karthesian coordinates of the resulting point on the hull.
			*/
			virtual Vector3d hullIntersectionFromKarthPointer(const Vector3d& karthPointer);

			/**
				\brief Calculates the coordinates of a point on this spheres's hull, given the spheric coordinates that intersect with the hull.
				\param[in] paramPointer Spheric coordinates pointing to the shapes hull.
				\returns Karthesian coordinates of the resulting point on the hull.
				\note Order of spheric coordinates in the given 3x1 vector are: radius, theta, phi. The radius parameter is ignored.
			*/
			virtual Vector3d hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer);

			/**
				\brief Packs the spheres's internal information into a formatted readable string.
			*/
			const virtual string toString() const;

			/**
				\brief Get the radius of this sphere.
				\returns Radius of this sphere.
			*/
			inline const double& getRadius() const { return radius; };
		protected:

			/**
				\brief Constructor a Sphere-object.
				\param[in] radius Radius of the sphere.
			*/
			Sphere(const double& radius);

			/*Radius of this sphere.*/
			double radius;
		private:

			/**
				\brief Calculates the bounding box of this sphere and stores it internally.
			*/
			virtual void calcBoundingBox();
		};

		/**
			\brief A class that implements the shape interface to represent cylinders of any size.
		*/
		class Cylinder : public Shape {
		public:

			/**
				\brief Call to instantiate a new Cylinder-object.
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" a Cylinder-object, i.e. Constructor is private!
				\param[in] radius Radius of the cylinder.
				\param[in] length Total length of the cylinder, which is z-Axis aligned.
			*/
			static Cylinder* create(const double& radius, const double& length);

			/**
				\brief Calculate the parametrized coordinates of a cylinder from given karthesian coordinates.
				\param[in] karth Karthesian coordinates that should be transformed to cylindric coordinates.
				\returns Cylindric coordinates from given karthesian coordinates.
				\note Order of cylindric coordinates in the returned 3x1 vector are: radius, phi, z.
			*/
			virtual Vector3d karthesianToParametrizedCoordinates(const Vector3d& karth);
			
			/**
				\brief Calculate the karthesian coordinates from given parametrized coordinates of a cylinder.
				\param[in] param Cylindric coordinates that should be transformed to karthesian coordinates.
				\returns Karthesian coordinates from given cylindric coordinates.
				\note Order of cylindric coordinates in the given 3x1 vector are: radius, phi, z.
			*/
			virtual Vector3d parametrizedToKarthesianCoordinates(const Vector3d& param);

			/**
				\brief Calculates the coordinates of a point on this cylinder's hull, given a 3x1 vector
				in karth. coordinates pointing from the cylinder's origin to the point on the hull.
				\param[in] karthPointer Karthesian 3x1 vector pointing to the cylinder's hull.
				\returns Karthesian coordinates of the resulting point on the hull.
			*/
			virtual Vector3d hullIntersectionFromKarthPointer(const Vector3d& karthPointer);

			/**
				\brief Calculates the coordinates of a point on this cylinder's hull, given the cylindric coordinates that intersect with the hull.
				\param[in] paramPointer Cylindric coordinates pointing to the shapes hull.
				\returns Karthesian coordinates of the resulting point on the hull.
				\note Order of cylindric coordinates in the given 3x1 vector are: radius, phi, z. The radius parameter is ignored.
			*/
			virtual Vector3d hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer);

			/**
				\brief Packs the cylinder's internal information into a formatted readable string.
			*/
			const virtual string toString() const;

			/**
				\brief Get the radius of this cylinder.
				\returns Radius of this cylinder.
			*/
			const double& getRadius() const { return radius; };

			/**
				\brief Get the length of this cylinder.
				\returns Length of this cylinder.
			*/
			const double& getLength() const { return length; };
		protected:

			/**
				\brief Constructor a Cylinder-object.
				\param[in] radius Radius of the cylinder.
				\param[in] length Total length of the cylinder, which is z-Axis aligned.
			*/
			Cylinder(const double& radius, const double& length);

			/*Radius and length of this cylinder.*/
			double radius, length;
		private:

			/**
				\brief Calculates the bounding box of this cylinder and stores it internally.
			*/
			virtual void calcBoundingBox();
		};

		/**
			\brief A class that implements the shape interface to represent ellipsoids of any size.
		*/
		class Ellipsoid : public Shape {
		public:

			/**
				\brief Call to instantiate a new Ellipsoid-object.
				\returns Returns a smart pointer to the new object on the heap.
				\note This is the only way to "make" a Ellipsoid-object, i.e. Constructor is private!
				\param[in] rx Radius of the ellipsoid aligned to the x-Axis.
				\param[in] ry Radius of the ellipsoid aligned to the y-Axis.
				\param[in] rz Radius of the ellipsoid aligned to the z-Axis.
			*/
			static Ellipsoid* create(const double& rx, const double& ry, const double& rz);

			/**
				\brief Calculate the parametrized coordinates of an ellipsoid from given karthesian coordinates.
				\param[in] karth Karthesian coordinates that should be transformed to ellipsoidal coordinates.
				\returns Ellipsoidal coordinates from given karthesian coordinates.
				\note Order of ellipsoidal coordinates in the returned 3x1 vector are: radius, theta, phi, where radius (for now) is ignored.
				Here ellipsoidal coordinates are equal to spheric coordinates for simplicity reasons.
			*/
			virtual Vector3d karthesianToParametrizedCoordinates(const Vector3d& karth);

			/**
				\brief Calculate the karthesian coordinates from given parametrized coordinates of an ellipsoid.
				\param[in] param Ellipsoidal coordinates that should be transformed to karthesian coordinates.
				\returns Karthesian coordinates from given ellipsoidal coordinates.
				\note Order of ellipsoidal coordinates in the given 3x1 vector are: radius, theta, phi, where radius (for now) is ignored.
				Here ellipsoidal coordinates are equal to spheric coordinates for simplicity reasons.
			*/
			virtual Vector3d parametrizedToKarthesianCoordinates(const Vector3d& param);

			/**
				\brief Calculates the coordinates of a point on this ellipsoid's hull, given a 3x1 vector
				in karth. coordinates pointing from the ellipsoid's origin to the point on the hull.
				\param[in] karthPointer Karthesian 3x1 vector pointing to the ellipsoid's hull.
				\returns Karthesian coordinates of the resulting point on the hull.
				\note Here ellipsoidal coordinates are equal to spheric coordinates for simplicity reasons.
			*/
			virtual Vector3d hullIntersectionFromKarthPointer(const Vector3d& karthPointer);

			/**
				\brief Calculates the coordinates of a point on this ellipsoid's hull, given the ellipsoid coordinates that intersect with the hull.
				\param[in] paramPointer Ellipsoidal coordinates pointing to the shapes hull.
				\returns Karthesian coordinates of the resulting point on the hull.
				\note Order of ellipsoidal coordinates in the given 3x1 vector are: radius, theta, phi, where radius (for now) is ignored.
				Here ellipsoidal coordinates are equal to spheric coordinates for simplicity reasons.
			*/
			virtual Vector3d hullIntersectionFromParametrizedPointer(const Vector3d& paramPointer);

			/**
				\brief Packs the ellipsoid's internal information into a formatted readable string.
			*/
			const virtual string toString() const;

			/**
				\brief Get the radius of this ellipsoid that is x-Axis aligned.
				\returns X-Axis aligned radius.
			*/
			const double& getRadiusX() const { return rx; };

			/**
				\brief Get the radius of this ellipsoid that is y-Axis aligned.
				\returns Y-Axis aligned radius.
			*/
			const double& getRadiusY() const { return ry; };

			/**
				\brief Get the radius of this ellipsoid that is z-Axis aligned.
				\returns Z-Axis aligned radius.
			*/
			const double& getRadiusZ() const { return rz; };
		protected:

			/**
				\brief Constructor an Ellipsoid-object.
				\param[in] rx Radius of the ellipsoid aligned to the x-Axis.
				\param[in] ry Radius of the ellipsoid aligned to the y-Axis.
				\param[in] rz Radius of the ellipsoid aligned to the z-Axis.
			*/
			Ellipsoid(const double& rx, const double& ry, const double& rz);

			/* Radiuses of this ellipsoid aligned to their corresponding axis. */
			double rx, ry, rz;
		private:

			/**
				\brief Calculates the bounding box of this ellipsoid and stores it internally.
			*/
			virtual void calcBoundingBox();
		};

		// Aliases for used namespaces
		using std::shared_ptr;
		using ShapePtr = shared_ptr<Shape>;

		/**
			\brief Factory class to produce instances of shape objects, i.e. Sphere-objects, Cylinder-objects
			and Ellipsoid-objects. The type of the produced object depends on the given enumerator.
			Resulting objects are placed on the heap, hence the factory returns smart pointers only.
		*/
		class ShapeFactory {
		private:

			/**
				\brief Internal function to produce a sphere type object.
				\param[in] radius Radius of the new sphere.
				\returns Smart pointer to the new sphere object.
			*/
			static ShapePtr createSphere(const double& radius);

			/**
				\brief Internal function to produce a cylinder type object.
				\param[in] radius Radius of the new cylinder.
				\param[in] length Total length of the new cylinder.
				\returns Smart pointer to the new cylinder object.
			*/
			static ShapePtr createCylinder(const double& radius, const double& length);

			/**
				\brief Internal function to produce a ellipsoid type object.
				\param[in] rx Radius of the ellipsoid aligned to the x-Axis.
				\param[in] ry Radius of the ellipsoid aligned to the y-Axis.
				\param[in] rz Radius of the ellipsoid aligned to the z-Axis.
				\returns Smart pointer to the new ellipsoid object.
			*/
			static ShapePtr createEllipsoid(const double& rx, const double& ry, const double& rz);

			/**
				\brief Internal wrapper function for shapes that require 3 parameters, which calls
				the appropriate internal shape creation function, specified through the given enumerator.
				\param[in] shapeType Shape type enumerator defining the required shape.
				\param[in] a First param.
				\param[in] b Second param.
				\param[in] c Third param.
				\throws Exception if the given enumerator is unknown.
				\throws Exception if the given enumerator does no fit the amount of given parameters.
				\returns Smart pointer to the newly produced object.
			*/
			static ShapePtr createInternal(const ShapeType& shapeType, const double& a, const double& b, const double& c);

			/**
				\brief Internal wrapper function for shapes that require 2 parameters, which calls
				the appropriate internal shape creation function, specified through the given enumerator.
				\param[in] shapeType Shape type enumerator defining the required shape.
				\param[in] a First param.
				\param[in] b Second param.
				\throws Exception if the given enumerator is unknown.
				\throws Exception if the given enumerator does no fit the amount of given parameters.
				\returns Smart pointer to the newly produced object.
			*/
			static ShapePtr createInternal(const ShapeType& shapeType, const double& a, const double& b);

			/**
				\brief Internal wrapper function for shapes that require 1 parameter, which calls
				the appropriate internal shape creation function, specified through the given enumerator.
				\param[in] shapeType Shape type enumerator defining the required shape.
				\param[in] a First param.
				\throws Exception if the given enumerator is unknown.
				\throws Exception if the given enumerator does no fit the amount of given parameters.
				\returns Smart pointer to the newly produced object.
			*/
			static ShapePtr createInternal(const ShapeType& shapeType, const double& a);

		public:

			/**
				\brief Public template function that calls the appropriate internal wrapper function
				to create a new shape object, depending on the number of given arguments.
				\param[in] shapeType Shape type enumerator defining the required shape.
				\param[in] Args... Any number of parameters of the type double.
				\throws Exception if the given enumerator is unknown.
				\throws Exception if the given enumerator does no fit the amount of given parameters.
				\returns Smart pointer to the newly produced object.
			*/
			template<typename... Args>
			static ShapePtr create(const ShapeType& shapeType, Args... args)
			{
				return createInternal(shapeType, args...);
			}
		};

	}
}