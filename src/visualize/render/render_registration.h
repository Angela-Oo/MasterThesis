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
	ONLY_DEFORMATION_GRAPH,
	DEFORMATION,
	TARGET,
	ALL
};

std::string renderMeshModeToString(Render render_type);

class RenderRegistration {
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;
private:
	unsigned int _last_rendered_current_frame;
	unsigned int _last_rendered_reference_frame;
	bool _reigistration_finished;
public:
	unsigned int _current_frame;
	RegistrationType _registration_type;
	bool _render_points = true;
	Render _render_mesh = Render::ALL;
	bool _render_reference_mesh;
	bool _render_error;
	bool _render_deformation_graph;
private:
	void renderDeformedSourceMesh(const SurfaceMesh & deformed_points, bool override = false, bool debug_normals = false);
	void renderTargetMesh(const SurfaceMesh & target, bool override = false, bool debug_normals = false);
	void renderDeformationGraph(SurfaceMesh & deformation_graph, bool debug_normals = false);
public:
	Render nextRenderMeshMode();
	void saveCurrentWindowAsImage(std::string folder, std::string filename);
public:
	void render(ml::Cameraf& camera);
	void renderMesh(std::string id, SurfaceMesh & mesh, ml::RGBColor color);
	void renderCurrentFrame(std::shared_ptr<IMeshReader> mesh_reader, bool visible);
	void renderSelectedFrames(std::shared_ptr<IMeshReader> mesh_reader, std::vector<unsigned int> selected_frames);
	void renderReference(std::shared_ptr<IMeshReader> mesh_reader);
	void renderRegistration(std::shared_ptr<IRegistration> registration);
	void renderRegistrationSequence(std::shared_ptr<SequenceRegistration> registration);
	void renderError(std::vector<std::pair<Point, Point>> error_points);
public:
	RenderRegistration(ml::GraphicsDevice * graphics);
};
