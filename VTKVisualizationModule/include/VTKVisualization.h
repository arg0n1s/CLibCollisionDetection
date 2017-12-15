#pragma once

#include <vtk-8.0\vtkSmartPointer.h>
#include <vtk-8.0\vtkRenderer.h>
#include <vtk-8.0\vtkRenderWindow.h>
#include <vtk-8.0\vtkRenderWindowInteractor.h>

/*
#include <vtk-8.0\vtkCylinderSource.h>
#include <vtk-8.0\vtkSphereSource.h>
#include <vtk-8.0\vtkPolyDataMapper.h>
#include <vtk-8.0\vtkActor.h>

#include <vtk-8.0\vtkVertexGlyphFilter.h>
#include <vtk-8.0\vtkProperty.h>
#include <vtk-8.0\vtkAxesActor.h>
#include <vtk-8.0\vtkTransform.h>
#include <vtk-8.0\vtkPointData.h>
*/

namespace vis {

	// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

	class VTKVisualization {
	public:
		VTKVisualization();
	private:
		vtkSmartPointer<vtkRenderer> renderer;
		vtkSmartPointer<vtkRenderWindow> renderWindow;
		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

		void createRenderer();

		void createRenderWindow();

		void createRenderWindowInteractor();

	};


}