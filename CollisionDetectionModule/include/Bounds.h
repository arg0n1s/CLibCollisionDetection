#pragma once

namespace collision {
	namespace octtree {

		/**
		\brief Helper class to remove the number of parameters when defining
		spatial boundaries (aka. Bounding-Box) of a shape that should be inserted
		into the OctTree.
		To define boundaries two Bounds-objects are required, one for defining the
		upper bound and the other for defining the lower bounds of an object.
		*/
		class Bounds {
		public:

			/* Karthesian coordinates of the lower or upper bound.*/
			double x, y, z;

			/**
			\brief Default-Constructor
			*/
			Bounds() {};

			/**
			\brief Construct a Bounds-object representing the lower or upper bounds
			of a shape.
			\param[in] x Max. or min. boundary in x direction of the shape.
			\param[in] y Max. or min. boundary in y direction of the shape.
			\param[in] z Max. or min. boundary in z direction of the shape.
			*/
			Bounds(const double& x, const double& y, const double& z) : x(x), y(y), z(z) {};
		};

	}
}
