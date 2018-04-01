#pragma once

#include <iostream>
#include <memory>
#include <Eigen\Core>
#include <Eigen\Geometry>

namespace simobj {

	// Aliases for used namespaces
	using std::string;
	using Eigen::Vector3d;
	using Quaternion = Eigen::Quaternion<double>;

	namespace frames {

		/**
		\brief Enumerator used to signal coordinate transformation.
		*/
		enum ReferenceFrame {
			/*Local coordinates, to local coordinates.*/
			Local, 
			/*Local coordinates, to global coordinates.*/
			Global
		};

	}

	// Typedef for ease of use
	typedef frames::ReferenceFrame ReferenceFrame;

	/**
		\brief This abstract class must be implemented by all classes providing objects
		that may be used in the simulation, i.e. are "living" in the simulation container.
		Objects with this super-type have some kind of positon and orientation in real vector space,
		as well as unique ids as means of global identification within the simulation.
		\note Every class implementing this interface must provide a string representation of its
		internal data and implement means to transfrom local to global coordinates.
	*/
	class SimulationObject {
	public:
		// Forced memory alignment for older processor architectures.
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Typedefs for ease of use, i.e. aliases for SimulationObject smart pointer types.
		typedef std::shared_ptr<SimulationObject> SimObjPtr;
		typedef std::shared_ptr<const SimulationObject> SimObjConstPtr;
		typedef std::weak_ptr<SimulationObject> SimObjWeakPtr;
		typedef std::unique_ptr<SimulationObject> SimObjUniquePtr;

		/**
			\brief  Objects pack their internal information into a formatted readable string.
		*/
		virtual string toString() const = 0;

		/**
			\brief Get the local orientation of this object.
			\returns The objects local orientation as quaternion.
		*/
		virtual const Quaternion& getOrientation() const = 0;

		/**
			\brief Get the local position of this object.
			\returns The objects local position as a 3x1 vector in real space.
		*/
		virtual const Vector3d& getPosition() const = 0;

		/**
			\brief Get the karthesian coordinates of this object in global or local coordinates.
			\param[in] frame reference frame enumerator (i.e.: Return global or local coordinates?)
		*/
		virtual const Vector3d getPosition(const ReferenceFrame& frame) const = 0;

		/**
			\brief Get the orientation of this object as a quaternion, in global or local coordinates.
			\param[in] frame reference frame enumerator (i.e.: Return global or local orientation?)
		*/
		virtual const Quaternion getOrientation(const ReferenceFrame& frame) const = 0;

		/**
			\brief Get the type of this object.
			\returns The objects type as string.
		*/
		const string& getType() const;

		/**
			\brief Get the unique id of this object.
			\returns The objects id as unsigned long.
		*/
		const unsigned long& getId() const;

		/**
		\brief Rotates this object locally.
		\param[in] rotation Quaternion with which this object should be rotated.
		*/
		void rotate(const Quaternion& rotation);

		/**
		\brief Moves (translation) this object locally.
		\param[in] translation Vector that points from this object to the destination.
		*/
		void move(const Vector3d& translation);

		/**
			\brief Set the local orientation of this object.
			\param[in] The objects new local orientation as quaternion.
		*/
		void setOrientation(const Quaternion& orientation);

		/**
			\brief Set the local position of this object.
			\param[in] The objects new local position as a 3x1 vector in real space.
		*/
		void setPosition(const Vector3d& position);

	protected:

		/*This objects type.*/
		string type;

		/*This objects unique id.*/
		unsigned long id;

		/*This objects local position as a 3x1 vector in real space.*/
		Vector3d position;

		/*This objects local orientation as quaternion.*/
		Quaternion orientation;

		/** \brief Constructor
			\param[in] id Unique identifier of this object (used for reference in the simulation)
			\param[in] type Defines the type of this object (used to generate objects from a template)
		*/
		SimulationObject(const unsigned long& id, const string& type);

		/**
			\brief Destructor
		*/
		virtual ~SimulationObject();

	};
}