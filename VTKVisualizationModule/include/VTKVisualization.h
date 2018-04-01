#pragma once

#include <memory>
#include <Eigen\Core>
#include <Eigen\Dense>
#include <Eigen\Geometry>

#ifdef VTKVISULIZATIONMODULE_EXPORTS
#define VTK_VISUALIZATION_API __declspec(dllexport) 
#else
#define VTK_VISUALIZATION_API __declspec(dllimport) 
#endif

// VTK forward declaration to reduce the number of includes, i.e. reduce compile time!
template <typename T>
class vtkSmartPointer;

// VTK forward declarations to reduce the number of includes, i.e. reduce compile time!
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;

// Simulation Objects Module forward declarations to reduce the number of includes.
namespace simobj {
	class SimulationObject;
	namespace shapes {
		class Shape;
	}
}

// Collision Detection Module forward declarations to reduce the number of includes.
namespace collision {
	namespace octtree {
		template <typename T>
		class OctTree;

	}
}

namespace vis {

	// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

	// Aliases for used namespaces
	using simobj::SimulationObject;
	using simobj::shapes::Shape;
	using std::shared_ptr;
	using SimObjPtr = shared_ptr<SimulationObject>;
	using ShapePtr = shared_ptr<Shape>;
	using collision::octtree::OctTree;
	using TreePtr = shared_ptr<OctTree<unsigned int>>;
	using RenderPtr = shared_ptr<vtkSmartPointer<vtkRenderer> >;
	using RenderWindowPtr = shared_ptr<vtkSmartPointer<vtkRenderWindow> >;
	using RenderWindowInteractorPtr = shared_ptr<vtkSmartPointer<vtkRenderWindowInteractor> >;
	using Eigen::Vector3d;
	using Quaternion = Eigen::Quaternion<double>;

	/**
		\brief Objects of this class are used to render and display agents, agent clusters
		and collision trees from the CLib Collision Detection library.
		The user may rotate or move objects, change the camera angle and zoom inside the render window,
		beyond that no further interaction with displayed objects is implemented.
		VTK is used as the Back-End library for rendering and displaying basic geometric shapes,
		e.g. spheres, boxes, clyinders, etc.
	*/
	class VTKVisualization {
	public:

		/**
			\brief Intialize VTK Renderer-, RenderWindow- and RenderWindowInteractor-objects
			required to work properly.
		*/
		VTK_VISUALIZATION_API VTKVisualization();

		/**
			\brief Display rendered objects in a render window, with basic interactions
			like rotation or movement of objects, changing the camera angle and zoom.
		*/
		VTK_VISUALIZATION_API void display();

		/**
			\brief Render a single agent, from the collision library and store internally.
			Agent will be displayed upon calling the display() method.
			\param[in] agent Smart pointer an Agent-object.
		*/
		VTK_VISUALIZATION_API void renderAgent(SimObjPtr agent);

		/**
			\brief Render an agent cluster, from the collision library and store internally.
			The cluster and all contained agents will be displayed upon calling the display() method.
			\param[in] cluster Smart pointer an AgentCluster-object.
		*/
		VTK_VISUALIZATION_API void renderAgentCluster(SimObjPtr cluster);

		/**
			\brief Render a collision tree (aka. OctTree), from the collision library and store internally.
			Tree nodes will be rendered as collection of wireframe boxes containing agents from the cluster from
			which this tree was build. The tree and all contained agents will be displayed upon calling the display() method.
			\param[in] cluster Smart pointer an AgentCluster-object.
			\param[in] tree Smart pointer an OctTree-object representing the given cluster.
		*/
		VTK_VISUALIZATION_API void renderCollisionTree(SimObjPtr cluster, TreePtr tree);

			/* Flag to turn rendering of the cluster coordinate system on/off. */
		bool renderAxisOfClusterOn, 
			/* Flag to turn rendering of agent coordinate systems on/off. */
			renderAxisOfAgentOn, 
			/* Flag to turn rendering of empty OctTree nodes on/off. */
			renderEmptyNodesOn, 
			/* Flag to turn rendering of individual agent bounding boxes on/off. */
			renderBoundingBoxesOn,
			/* Flag to turn rendering of a global coordinate system on/off. */
			renderGlobalAxisOn;
	private:

		/* Smart pointer to a VTK Renderer-object. */
		RenderPtr renderer;
		/* Smart pointer to a VTK RenderWindow-object. */
		RenderWindowPtr renderWindow;
		/* Smart pointer to a VTK RenderWindowInteractor-object. */
		RenderWindowInteractorPtr renderWindowInteractor;

		/**
			\brief Create new VTK Renderer-object on the heap and store a pointer to it internally.
			This method also sets default background color, camera position and fps promt callback.
		*/
		void createRenderer();

		/**
			\brief Create new VTK RenderWindow-object on the heap and store a pointer to it internally.
			This method also sets default window size.
		*/
		void createRenderWindow();

		/**
			\brief Create new VTK RenderWindowInteractor-object on the heap and store a pointer to it internally.
		*/
		void createRenderWindowInteractor();

		/**
			\brief Render a single site of an agent, from the collision library and store internally.
			Site will be displayed upon calling the display() method.
			\param[in] site Smart pointer a Site-object.
		*/
		void renderSite(SimObjPtr site);

		/**
			\brief Render the axis representation of a global coordinate system and store internally.
			Axis will be displayed upon calling the display() method.
		*/
		void renderGlobalAxis();

		/**
			\brief Render the axis representation of a local cluster coordinate system and store internally.
			Axis will be displayed upon calling the display() method.
			\param[in] cluster Smart pointer an AgentCluster-object.
		*/
		void renderClusterAxis(SimObjPtr cluster);

		/**
			\brief Render the axis representation of a local agent coordinate system and store internally.
			Axis will be displayed upon calling the display() method.
			\param[in] agent Smart pointer an Agent-object.
		*/
		void renderAgentAxis(SimObjPtr agent);

		/**
			\brief Render the bounding box agent's shape store internally.
			BBox will be displayed upon calling the display() method, if BBox flag is true.
			\param[in] agent Smart pointer an Agent-object.
		*/
		void renderAgentBBox(SimObjPtr agent);

		/**
			\brief Render the OctTrees nodes as boxes and store internally.
			If the display empty nodes flag is true the complete tree will be rendered.
			Tree will be displayed upon calling the display() method.
			\param[in] tree Smart pointer an OctTree-object representing a cluster.
		*/
		void renderTree(TreePtr tree);

		/**
			\brief Wrapper method for rendering a generic shape, defined by the ShapeType enumerator
			in the given shape type object. The wrapper will call the appropriate methods to draw
			specific shapes.
			\param[in] position Origin of the shape as a 3x1 vector.
			\param[in] orientation Rotation of the shape as a quaternion.
			\param[in] shape Smart pointer to the shape object to be rendered.
		*/
		void renderShape(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);

		/**
			\brief Specific method for rendering a spheric shape, defined by given shape object.
			\param[in] position Origin of the shape as a 3x1 vector.
			\param[in] orientation Rotation of the shape as a quaternion.
			\param[in] shape Smart pointer to the shape object to be rendered.
		*/
		void renderSphere(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);

		/**
			\brief Specific method for rendering a cylindric shape, defined by given shape object.
			\param[in] position Origin of the shape as a 3x1 vector.
			\param[in] orientation Rotation of the shape as a quaternion.
			\param[in] shape Smart pointer to the shape object to be rendered.
		*/
		void renderCylinder(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);

		/**
			\brief Specific method for rendering an ellipsoidal shape, defined by given shape object.
			\param[in] position Origin of the shape as a 3x1 vector.
			\param[in] orientation Rotation of the shape as a quaternion.
			\param[in] shape Smart pointer to the shape object to be rendered.
		*/
		void renderEllipsoid(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);

		/**
			\brief Helper function used to transform Quaternions to Euler-Angles (more specifically Cardan-Angles).
			\param[in] quat Rotation as a quaternion.
			\param[out] r Roll: Rotation about the x-axis.
			\param[out] p Pitch: Rotation about the y-axis.
			\param[out] y Yaw: Rotation about the z-axis.
		*/
		static void fromQuatToEuler(const Quaternion& quat, double& r, double& p, double& y);
	};


}