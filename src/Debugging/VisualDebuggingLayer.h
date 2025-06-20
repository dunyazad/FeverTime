#pragma once

#include <Common.h>
#include <Color.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

class VisualDebuggingLayer
{
public:
	enum Representation { HPoints, HWireFrame, HSurface };

	VisualDebuggingLayer(const string& layerName);
	~VisualDebuggingLayer();

	void Initialize(vtkSmartPointer<vtkRenderer> renderer);
	void Terminate();

	void AddPoint(const Eigen::Vector3f& p, const Color4& color);

	void AddLine(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Color4& color);
	
	void AddTriangle(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Color4& color);

	void AddPlane(const Eigen::Vector3f& center, const Eigen::Vector3f& scale, const Eigen::Vector3f& normal, const Color4& color);

	void AddSphere(const Eigen::Vector3f& center, const Eigen::Vector3f& scale, const Eigen::Vector3f& normal, const Color4& color);
	
	void AddCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale, const Eigen::Vector3f& normal, const Color4& color);

	void AddGlyph(const Eigen::Vector3f& center, const Eigen::Vector3f& scale, const Eigen::Vector3f& normal, const Color4& color);

	void AddArrow(const Eigen::Vector3f& center, const Eigen::Vector3f& normal, float scale, const Color4& color);

	void AddWiredBox(const Eigen::Vector3f& boxMin, const Eigen::Vector3f& boxMax, const Color4& color);

	void Update();
	void Clear();

	void ShowAll(bool show);
	void ToggleVisibilityAll();
	void SetRepresentationAll(Representation representation);
	void ToggleAllRepresentation();

	void ShowPoints(bool show);
	void TogglePoints();
	void SetRepresentationPoints(Representation representation);
	void TogglePointsRepresentation();
	
	void ShowLines(bool show);
	void ToggleLines();
	void SetRepresentationLines(Representation representation);
	void ToggleLinesRepresentation();
	
	void ShowTriangles(bool show);
	void ToggleTriangles();
	void SetRepresentationTriangles(Representation representation);
	void ToggleTrianglesRepresentation();
	
	void ShowPlanes(bool show);
	void TogglePlanes();
	void SetRepresentationPlanes(Representation representation);
	void TogglePlanesRepresentation();

	void ShowSpheres(bool show);
	void ToggleSpheres();
	void SetRepresentationSpheres(Representation representation);
	void ToggleSpheresRepresentation();

	void ShowCubes(bool show);
	void ToggleCubes();
	void SetRepresentationCubes(Representation representation);
	void ToggleCubesRepresentation();
	
	void ShowArrows(bool show);
	void ToggleArrows();
	void SetRepresentationArrows(Representation representation);
	void ToggleArrowsRepresentation();

	void ShowWiredBoxes(bool show);
	void ToggleWiredBoxes();
	void SetRepresentationWiredBoxes(Representation representation);
	void ToggleWiredBoxesRepresentation();

	float GetPointSize();
	void SetPointSize(float size);
	float GetLineWidth();
	void SetLineWidth(float width);

	inline vtkSmartPointer<vtkActor> GetPointActor() { return pointActor; }
	inline vtkSmartPointer<vtkActor> GetLineActor() { return lineActor; }
	inline vtkSmartPointer<vtkActor> GetTriangleActor() { return triangleActor; }
	inline vtkSmartPointer<vtkActor> GetPlaneActor() { return planeActor; }
	inline vtkSmartPointer<vtkActor> GetSphereActor() { return sphereActor; }
	inline vtkSmartPointer<vtkActor> GetCubeActor() { return cubeActor; }
	inline vtkSmartPointer<vtkActor> GetGlyphActor() { return glyphActor; }
	inline vtkSmartPointer<vtkActor> GetArrowActor() { return arrowActor; }
	inline vtkSmartPointer<vtkActor> GetWiredBoxActor() { return wiredBoxActor; }

protected:
	inline void ShowActor(vtkSmartPointer<vtkRenderer> renderer, vtkSmartPointer<vtkActor> actor, bool show)
	{
		if (nullptr != actor)
		{
			actor->SetVisibility(show);
			//renderer->GetRenderWindow()->Render();
		}
	}

	inline void ToggleActorVisibility(vtkSmartPointer<vtkRenderer> renderer, vtkSmartPointer<vtkActor> actor)
	{
		if (nullptr != actor)
		{
			actor->SetVisibility(!actor->GetVisibility());
			//renderer->GetRenderWindow()->Render();
		}
	}

	inline void SetActorRepresentation(vtkSmartPointer<vtkRenderer> renderer, vtkSmartPointer<vtkActor> actor, Representation representation)
	{
		if (nullptr != actor)
		{
			actor->GetProperty()->SetRepresentation(representation);
			//renderer->GetRenderWindow()->Render();
		}
	}

	inline void ToggleActorRepresentation(vtkSmartPointer<vtkRenderer> renderer, vtkSmartPointer<vtkActor> actor)
	{
		if (nullptr != actor)
		{
			auto mode = actor->GetProperty()->GetRepresentation();
			mode += 1;
			if (mode > VTK_SURFACE)
			{
				mode = VTK_POINTS;
			}
			actor->GetProperty()->SetRepresentation(mode);
			//renderer->GetRenderWindow()->Render();
		}
	}

private:
	string layerName = "";
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renderWindow;

	vtkSmartPointer<vtkActor> pointActor;
	vtkSmartPointer<vtkPolyDataMapper> pointPolyDataMapper;
	vtkSmartPointer<vtkPolyData> pointPolyData;

	vtkSmartPointer<vtkActor> lineActor;
	vtkSmartPointer<vtkPolyDataMapper> linePolyDataMapper;
	vtkSmartPointer<vtkPolyData> linePolyData;

	vtkSmartPointer<vtkActor> triangleActor;
	vtkSmartPointer<vtkPolyDataMapper> trianglePolyDataMapper;
	vtkSmartPointer<vtkPolyData> trianglePolyData;

	vtkSmartPointer<vtkActor> planeActor;
	vtkSmartPointer<vtkGlyph3DMapper> planePolyDataMapper;
	vtkSmartPointer<vtkPolyData> planePolyData;

	vtkSmartPointer<vtkActor> sphereActor;
	vtkSmartPointer<vtkGlyph3DMapper> spherePolyDataMapper;
	vtkSmartPointer<vtkPolyData> spherePolyData;

	vtkSmartPointer<vtkActor> cubeActor;
	vtkSmartPointer<vtkGlyph3DMapper> cubePolyDataMapper;
	vtkSmartPointer<vtkPolyData> cubePolyData;

	vtkSmartPointer<vtkActor> glyphActor;
	vtkSmartPointer<vtkGlyph3DMapper> glyphPolyDataMapper;
	vtkSmartPointer<vtkPolyData> glyphPolyData;

	vtkSmartPointer<vtkActor> arrowActor;
	vtkSmartPointer<vtkPolyDataMapper> arrowPolyDataMapper;
	vtkSmartPointer<vtkGlyph3D> arrowGlyph3D;
	vtkSmartPointer<vtkPolyData> arrowPolyData;

	vtkSmartPointer<vtkActor> wiredBoxActor;
	vtkSmartPointer<vtkPolyDataMapper> wiredBoxPolyDataMapper;
	vtkSmartPointer<vtkGlyph3D> wiredBoxGlyph3D;
	vtkSmartPointer<vtkPolyData> wiredBoxPolyData;

	void DrawPoints();
	void DrawLines();
	void DrawTriangle();
	void DrawPlane();
	void DrawSpheres();
	void DrawCubes();
	void DrawGlyphs();
	void DrawArrows();
	void DrawWiredBoxes();

	vector<std::tuple<Eigen::Vector3f, Color4>> pointInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Color4>> lineInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Color4>> triangleInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Color4>> planeInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Color4>> sphereInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Color4>> cubeInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Color4>> glyphInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, float, Color4>> arrowInfosToDraw;
	vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f, Color4>> wiredBoxInfosToDraw;
};
