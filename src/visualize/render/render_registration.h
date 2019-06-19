#pragma once

#include "mLibInclude.h"
#include "pointsRenderer.h"
#include "meshRenderer.h"
#include "algo/registration/i_registration.h"
#include "algo/registration/sequence_registration/sequence_registration.h"

#include <memory>

enum Render
{
	NONE,
	DEFORMATION,
	TARGET,
	ALL
};

class RenderRegistration {
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;
public:
	RegistrationType _registration_type;
	bool _render_points = true;
	Render _render_mesh = Render::ALL;
	bool _render_reference_mesh;
	bool _render_error;
	bool _render_deformation_graph;
public:
	Render nextRenderMeshMode();
	void saveCurrentWindowAsImage(std::string folder, std::string filename);
public:
	void render(ml::Cameraf& camera);
	void renderMesh(std::string id, SurfaceMesh & mesh, ml::RGBColor color);
	void renderCurrentFrame(std::shared_ptr<IMeshReader> mesh_reader, unsigned int current_frame, bool visible);
	void renderSelectedFrames(std::shared_ptr<IMeshReader> mesh_reader, std::vector<unsigned int> selected_frames);
	void renderReference(std::shared_ptr<IMeshReader> mesh_reader, unsigned int current_frame);
	void renderRegistration(std::shared_ptr<IRegistration> registration);
	void renderRegistrationSequence(std::shared_ptr<SequenceRegistration> registration);
	void renderError(std::vector<std::pair<Point, Point>> error_points);
public:
	RenderRegistration(ml::GraphicsDevice * graphics);
};
