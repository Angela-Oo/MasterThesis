#include "render_sequence_registration.h"

namespace Visualizer {


void RendererRegistration::renderDeformedSourceMesh(const SurfaceMesh & deformed_mesh, RegistrationRenderMode mode)
{
	// render point clouds
	std::string deformed_id = "deformed_id";

	if (mode == RegistrationRenderMode::TARGET || mode == RegistrationRenderMode::ONLY_DEFORMATION_GRAPH) { // no deformed mesh
		_renderer->remove(deformed_id);
	}
	else {
		RenderMode render_mode = RenderMode::MESH;
		if (mode == RegistrationRenderMode::NONE) { // only render target as points
			render_mode = RenderMode::POINT;
		}
		_renderer->insert(deformed_id, deformed_mesh, render_mode, ml::RGBColor::Cyan, 0.001f, true);
	}
}

void RendererRegistration::renderTargetMesh(const SurfaceMesh & target, RegistrationRenderMode mode)
{
	std::string target_id = "target_id";
	if (mode == RegistrationRenderMode::ONLY_DEFORMATION_GRAPH) { // no target
		_renderer->remove(target_id);
	}
	else {
		RenderMode render_mode = RenderMode::MESH;
		if (mode == RegistrationRenderMode::NONE || mode == RegistrationRenderMode::DEFORMATION) { // only render target as points
			render_mode = RenderMode::POINT;
		}
		_renderer->insert(target_id, target, render_mode, ml::RGBColor::Green, 0.001f, true);
	}
}

void RendererRegistration::renderDeformationGraph(const SurfaceMesh & deformation_graph, RegistrationRenderMode mode)
{
	if (true) {
		_renderer->insert("deformation_graph", deformation_graph, RenderMode::EDGE, 0.001f, true);
	}
	else {
		_renderer->remove("deformation_graph");
	}
}

RendererRegistration::RendererRegistration(std::shared_ptr<Renderer> renderer)
	: _renderer(renderer)
{
}

}
