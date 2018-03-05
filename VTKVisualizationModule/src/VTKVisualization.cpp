#include <vtk-8.0\vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2); VTK_MODULE_INIT(vtkInteractionStyle); VTK_MODULE_INIT(vtkRenderingFreeType); VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

#include "..\include\VTKVisualization.h"

#include <vtk-8.0\vtkSmartPointer.h>
#include <vtk-8.0\vtkRenderer.h>
#include <vtk-8.0\vtkRenderWindow.h>
#include <vtk-8.0\vtkRenderWindowInteractor.h>

#include <vtk-8.0\vtkCubeSource.h>
#include <vtk-8.0\vtkSphereSource.h>
#include <vtk-8.0\vtkCylinderSource.h>
#include <vtk-8.0\vtkParametricEllipsoid.h>
#include <vtk-8.0\vtkParametricFunctionSource.h>
#include <vtk-8.0\vtkAxesActor.h>

#include <vtk-8.0\vtkPolyDataMapper.h>
#include <vtk-8.0\vtkActor.h>
#include <vtk-8.0\vtkProperty.h>
#include <vtk-8.0\vtkTransform.h>

#include <vtk-8.0\vtkCallbackCommand.h>
#include <vtk-8.0\vtkInteractorObserver.h>
#include <vtk-8.0\vtkGlyph3D.h>
#include <vtk-8.0\vtkFloatArray.h>
#include <vtk-8.0\vtkPointData.h>

#include <Agent.h>
#include <Site.h>
#include <AgentCluster.h>
#include <Shape.h>

#include <OctTree.h>

namespace vis {

	// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

	using simobj::Agent;
	using simobj::AgentCluster;
	using simobj::Site;
	using simobj::SimObjPtr;
	using simobj::ShapePtr;
	using simobj::shapes::ShapeType;
	using simobj::shapes::Sphere;
	using simobj::shapes::Cylinder;
	using simobj::shapes::Ellipsoid;
	using collision::octtree::OctTree;
	using collision::octtree::Bounds;
	using NodePtr = collision::octtree::NodePtr<unsigned int>;

	VTK_VISUALIZATION_API VTKVisualization::VTKVisualization() {
		createRenderer();
		createRenderWindow();
		createRenderWindowInteractor();
		renderAxisOfClusterOn = renderAxisOfAgentOn = renderEmptyNodesOn = renderBoundingBoxesOn = showFPSOn = false;
	}

	VTK_VISUALIZATION_API void VTKVisualization::display() {
		(*renderWindow)->Render();
		(*renderWindowInteractor)->Start();

		createRenderer();
		createRenderWindow();
		createRenderWindowInteractor();
	}

	VTK_VISUALIZATION_API void VTKVisualization::renderAgent(SimObjPtr agent) {
		shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent);
		renderShape(agtPtr->getPosition(simobj::ReferenceFrame::Global), agtPtr->getOrientation(simobj::ReferenceFrame::Global), agtPtr->getShape());
		for (auto site : agtPtr->getAllSites()) {
			shared_ptr<Site> stPtr = std::static_pointer_cast<Site>(site.second);
			renderSite(stPtr);
		}
		if (renderBoundingBoxesOn) {
			renderAgentBBox(agtPtr);
		}
		if (renderAxisOfAgentOn) {
			renderAgentAxis(agtPtr);
		}
	}

	VTK_VISUALIZATION_API void VTKVisualization::renderAgentCluster(SimObjPtr cluster) {
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		if (renderAxisOfClusterOn) {
			renderClusterAxis(clsPtr);
		}
		for (auto agent : clsPtr->getAllAgents()) {
			shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent.second);
			renderAgent(agtPtr);
		}
	}

	VTK_VISUALIZATION_API void VTKVisualization::renderCollisionTree(SimObjPtr cluster, TreePtr tree) {
		renderAgentCluster(cluster);
		renderTree(tree);
	}
	
	void VTKVisualization::renderAgentBBox(SimObjPtr agent) {
		shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent);
		const Quaternion& orientation = agtPtr->getOrientation(simobj::ReferenceFrame::Global);
		const Vector3d& position = agtPtr->getPosition(simobj::ReferenceFrame::Global);
		const simobj::shapes::BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
		vtkSmartPointer<vtkCubeSource> box = vtkSmartPointer<vtkCubeSource>::New();
		box->SetXLength(bbx.width);
		box->SetYLength(bbx.height);
		box->SetZLength(bbx.length);

		vtkSmartPointer<vtkPolyDataMapper> boxMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();

		boxMapper->GlobalImmediateModeRenderingOn();
		boxMapper->SetInputConnection(box->GetOutputPort());
		
		boxMapper->SetImmediateModeRendering(true);
		vtkSmartPointer<vtkActor> boxActor = vtkSmartPointer<vtkActor>::New();
		boxActor->SetMapper(boxMapper);
		// color is for now hard coded, this can be changed in the future 
		boxActor->GetProperty()->SetColor(0.2, 0.0, 0.2);
		

		vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
		transform->Translate(position.x(), position.y(), position.z());
		double r, p, y;
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		
		boxActor->SetUserTransform(transform);
		boxActor->GetProperty()->SetRepresentationToWireframe();
		(*renderer)->AddActor(boxActor);

	}

	void VTKVisualization::renderTree(TreePtr tree) {
		std::vector<NodePtr> nodes = tree->getNodes();

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkFloatArray> scales =
			vtkSmartPointer<vtkFloatArray>::New();
		scales->SetName("scales");

		vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();

		for (auto node : nodes) {
			if (node->isEmpty() && !renderEmptyNodesOn) continue;
			Bounds diameter = node->getDiameter();
			points->InsertNextPoint(node->getX(), node->getY(), node->getZ());
			scales->InsertNextValue(diameter.x);
		}

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);
		polydata->GetPointData()->SetScalars(scales);
		vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
		glyph3D->SetInputData(polydata);
		glyph3D->SetScaleModeToScaleByScalar();
		glyph3D->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyph3D->GetOutputPort());

		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetRepresentationToWireframe();
		actor->GetProperty()->SetColor(0.0, 0.8, 0.8);

		(*renderer)->AddActor(actor);
	}
	
	void VTKVisualization::renderSite(SimObjPtr site) {
		shared_ptr<Site> stPtr = std::static_pointer_cast<Site>(site);
		shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(stPtr->getOwner());
		
		const simobj::shapes::BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
		// for now scaling is hard coded, can be changed in the future
		double radius = std::max(std::max(bbx.width, bbx.height), bbx.length)/10.0;

		vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
		sphere->SetRadius(radius);
		
		vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		
		sphereMapper->GlobalImmediateModeRenderingOn();
		sphereMapper->SetInputConnection(sphere->GetOutputPort());
		sphereMapper->SetImmediateModeRendering(true);

		vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
		sphereActor->SetMapper(sphereMapper);
		// color is for now hard coded, this can be changed in the future 
		sphereActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
		Vector3d position = stPtr->getPosition(simobj::ReferenceFrame::Global);
		sphereActor->SetPosition(position.x(), position.y(), position.z());
		(*renderer)->AddActor(sphereActor);

	}

	void VTKVisualization::renderClusterAxis(SimObjPtr cluster)
	{
		shared_ptr<AgentCluster> clsPtr = std::static_pointer_cast<AgentCluster>(cluster);
		const Quaternion& orientation = clsPtr->getOrientation(simobj::ReferenceFrame::Global);
		const Vector3d& origin = clsPtr->getPosition(simobj::ReferenceFrame::Global);

		vtkSmartPointer<vtkAxesActor> axes =
			vtkSmartPointer<vtkAxesActor>::New();
		axes->SetDragable(0);
		axes->SetTotalLength(50, 50, 50);
		vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
		transform->Translate(origin.x(), origin.y(), origin.z());
		double r, p, y;
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		axes->SetUserTransform(transform);
		(*renderer)->AddActor(axes);
	}
	
	void VTKVisualization::renderAgentAxis(SimObjPtr agent)
	{
		shared_ptr<Agent> agtPtr = std::static_pointer_cast<Agent>(agent);
		const Quaternion& orientation = agtPtr->getOrientation(simobj::ReferenceFrame::Global);
		const Vector3d& origin = agtPtr->getPosition(simobj::ReferenceFrame::Global);

		const simobj::shapes::BoundingBox& bbx = agtPtr->getShape()->getBoundingBox();
		// for now scaling is hard coded, can be changed in the future
		double radius = std::max(std::max(bbx.width, bbx.height), bbx.length)*3;

		vtkSmartPointer<vtkAxesActor> axes =
			vtkSmartPointer<vtkAxesActor>::New();
		axes->SetDragable(0);
		axes->SetTotalLength(radius, radius, radius);
		if (agtPtr->isInAnyCluster()) {
			axes->AxisLabelsOff();
		}
		vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
		transform->Translate(origin.x(), origin.y(), origin.z());
		double r, p, y;
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		axes->SetUserTransform(transform);
		(*renderer)->AddActor(axes);
	}

	void VTKVisualization::renderShape(const Vector3d& position, const Quaternion& orientation, ShapePtr shape) {
		switch (shape->getType()) {
			case ShapeType::Sphere: {
				renderSphere(position, orientation, shape);
				break;
			}
			case ShapeType::Cylinder: {
				renderCylinder(position, orientation, shape);
				break;
			}
			case ShapeType::Ellipsoid: {
				renderEllipsoid(position, orientation, shape);
				break;
			}
			default: return;

		}
	}

	void VTKVisualization::renderSphere(const Vector3d& position, const Quaternion& orientation, ShapePtr shape) {
		shared_ptr<Sphere> sphPtr = std::static_pointer_cast<Sphere>(shape);

		vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
		sphere->SetRadius(sphPtr->getRadius());
		
		vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();

		sphereMapper->GlobalImmediateModeRenderingOn();
		sphereMapper->SetInputConnection(sphere->GetOutputPort());
		sphereMapper->SetImmediateModeRendering(true);

		vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
		sphereActor->SetMapper(sphereMapper);
		// color is for now hard coded, this can be changed in the future 
		sphereActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(position.x(), position.y(), position.z());
		double r, p, y;
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		sphereActor->SetUserTransform(transform);

		(*renderer)->AddActor(sphereActor);
	}

	void VTKVisualization::renderCylinder(const Vector3d& position, const Quaternion& orientation, ShapePtr shape) {
		shared_ptr<Cylinder> cylPtr = std::static_pointer_cast<Cylinder>(shape);
		vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
		// resolution is for now hard cored, this can be changed in the future
		cylinder->SetResolution(51);
		cylinder->SetHeight(cylPtr->getLength());
		cylinder->SetRadius(cylPtr->getRadius());
		
		vtkSmartPointer<vtkPolyDataMapper> cylinderMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();

		cylinderMapper->GlobalImmediateModeRenderingOn();
		cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
		cylinderMapper->SetImmediateModeRendering(true);

		vtkSmartPointer<vtkActor> cylinderActor = vtkSmartPointer<vtkActor>::New();
		cylinderActor->SetMapper(cylinderMapper);
		// color is for now hard coded, this can be changed in the future 
		cylinderActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(position.x(), position.y(), position.z());

		Vector3d v1(0, 1, 0);
		Vector3d v2(0, 0, 1);
		Quaternion correction = orientation*Quaternion::FromTwoVectors(v1, v2);

		double r, p, y;
		fromQuatToEuler(correction, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		/*
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		*/
		cylinderActor->SetUserTransform(transform);
		
		(*renderer)->AddActor(cylinderActor);
	}

	void VTKVisualization::renderEllipsoid(const Vector3d& position, const Quaternion& orientation, ShapePtr shape) {
		shared_ptr<Ellipsoid> ellPtr = std::static_pointer_cast<Ellipsoid>(shape);

		vtkSmartPointer<vtkParametricEllipsoid> ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
		ellipsoid->SetXRadius(ellPtr->getRadiusX());
		ellipsoid->SetYRadius(ellPtr->getRadiusY());
		ellipsoid->SetZRadius(ellPtr->getRadiusZ());

		vtkSmartPointer<vtkParametricFunctionSource> ellipsoidSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
		ellipsoidSource->SetParametricFunction(ellipsoid);
		ellipsoidSource->SetUResolution(51);
		ellipsoidSource->SetVResolution(51);
		ellipsoidSource->SetWResolution(51);
		ellipsoidSource->Update();
		
		vtkSmartPointer<vtkPolyDataMapper> ellipsoidMapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();

		ellipsoidMapper->GlobalImmediateModeRenderingOn();
		ellipsoidMapper->SetInputConnection(ellipsoidSource->GetOutputPort());
		ellipsoidMapper->SetImmediateModeRendering(true);

		vtkSmartPointer<vtkActor> ellipsoidActor = vtkSmartPointer<vtkActor>::New();
		ellipsoidActor->SetMapper(ellipsoidMapper);
		// color is for now hard coded, this can be changed in the future 
		ellipsoidActor->GetProperty()->SetColor(1.0, 0.0, 0.0);

		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(position.x(), position.y(), position.z());
		double r, p, y;
		fromQuatToEuler(orientation, r, p, y);
		transform->RotateX(r);
		transform->RotateY(p);
		transform->RotateZ(y);
		ellipsoidActor->SetUserTransform(transform);

		(*renderer)->AddActor(ellipsoidActor);
	}

	void CallbackFunction(vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData))
	{
		vtkRenderer* renderer = static_cast<vtkRenderer*>(caller);

		double timeInSeconds = renderer->GetLastRenderTimeInSeconds();
		double fps = 1.0 / timeInSeconds;
		std::cout << "FPS: " << fps << std::endl;

		std::cout << "Callback" << std::endl;
	}

	void VTKVisualization::createRenderer()
	{
		renderer = std::make_shared<vtkSmartPointer<vtkRenderer> >(vtkSmartPointer<vtkRenderer>::New());
		// background color is for now hard coded, this can be changed in the future 
		(*renderer)->SetBackground(0.2, 0.2, 0.2);

		if (showFPSOn) {
		vtkSmartPointer<vtkCallbackCommand> callback =
			vtkSmartPointer<vtkCallbackCommand>::New();

		callback->SetCallback(CallbackFunction);
		(*renderer)->AddObserver(vtkCommand::EndEvent, callback);
		}
	}

	void VTKVisualization::createRenderWindow()
	{
		renderWindow = std::make_shared<vtkSmartPointer<vtkRenderWindow> >(vtkSmartPointer<vtkRenderWindow>::New());
		(*renderWindow)->AddRenderer(*renderer);
	}

	void VTKVisualization::createRenderWindowInteractor()
	{
		renderWindowInteractor = std::make_shared<vtkSmartPointer<vtkRenderWindowInteractor> >(vtkSmartPointer<vtkRenderWindowInteractor>::New());
		(*renderWindowInteractor)->SetRenderWindow(*renderWindow);
	}

	void VTKVisualization::fromQuatToEuler(const Quaternion& quat, double& r, double& p, double& y) {
		double *eulerAngles = quat.toRotationMatrix().eulerAngles(0, 1, 2).data();
		eulerAngles[0] *= 180.0 / M_PI;
		eulerAngles[1] *= 180.0 / M_PI;
		eulerAngles[2] *= 180.0 / M_PI;
		r = eulerAngles[0];
		p = eulerAngles[1];
		y = eulerAngles[2];
	}


}