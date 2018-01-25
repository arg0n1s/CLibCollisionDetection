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

template <typename T>
class vtkSmartPointer;

class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;

namespace simobj {
	class SimulationObject;
	namespace shapes {
		class Shape;
	}
}

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

	class VTKVisualization {
	public:
		VTK_VISUALIZATION_API VTKVisualization();
		VTK_VISUALIZATION_API void display();
		VTK_VISUALIZATION_API void renderAgent(SimObjPtr agent);
		VTK_VISUALIZATION_API void renderAgentCluster(SimObjPtr cluster);
		VTK_VISUALIZATION_API void renderCollisionTree(SimObjPtr cluster, TreePtr tree);

		bool renderAxisOfClusterOn, renderAxisOfAgentOn, renderEmptyNodesOn, renderBoundingBoxesOn, showFPSOn;
	private:
		RenderPtr renderer;
		RenderWindowPtr renderWindow;
		RenderWindowInteractorPtr renderWindowInteractor;

		void createRenderer();
		void createRenderWindow();
		void createRenderWindowInteractor();

		void renderSite(SimObjPtr site);

		void renderClusterAxis(SimObjPtr cluster);
		void renderAgentAxis(SimObjPtr agent);
		void renderAgentBBox(SimObjPtr agent);
		void renderTree(TreePtr tree);

		void renderShape(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);
		void renderSphere(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);
		void renderCylinder(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);
		void renderEllipsoid(const Vector3d& position, const Quaternion& orientation, ShapePtr shape);

		static void fromQuatToEuler(const Quaternion& quat, double& r, double& p, double& y);
	};


}