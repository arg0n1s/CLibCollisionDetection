#include "..\include\VTKVisualization.h"

namespace vis {

	// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

	VTKVisualization::VTKVisualization() {
		createRenderer();
		createRenderWindow();
		createRenderWindowInteractor();
	}

	void VTKVisualization::createRenderer()
	{
		renderer = vtkSmartPointer<vtkRenderer>::New();
		renderer->SetBackground(0.2, 0.2, 0.2);
	}

	void VTKVisualization::createRenderWindow()
	{
		renderWindow =
			vtkSmartPointer<vtkRenderWindow>::New();
		renderWindow->AddRenderer(renderer);
	}

	void VTKVisualization::createRenderWindowInteractor()
	{
		renderWindowInteractor =
			vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);
	}


}