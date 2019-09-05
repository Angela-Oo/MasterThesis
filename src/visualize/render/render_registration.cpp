#include "render_registration.h"

namespace Visualizer {


void RendererRegistration::renderDeformedSourceMesh(const SurfaceMesh & deformed_mesh, RegistrationRenderMode mode, bool render_vertex_color)
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
		if(render_vertex_color)
		{
			_renderer->insert(deformed_id, deformed_mesh, render_mode, true, 0.001f);
		}
		else {
			_renderer->insert(deformed_id, deformed_mesh, render_mode, ml::RGBColor::Cyan, true, 0.001f);
		}
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
		_renderer->insert(target_id, target, render_mode, ml::RGBColor::Green, true, 0.001f);
	}
}

void RendererRegistration::renderDeformationGraph(const SurfaceMesh & deformation_graph, RegistrationRenderMode mode)
{
	std::string deformation_graph_id = "deformation_graph";
	if (true) {
		_renderer->insert(deformation_graph_id, deformation_graph, RenderMode::EDGE, true, 0.001f);
	}
	else {
		_renderer->remove(deformation_graph_id);
	}
}

RendererRegistration::RendererRegistration(std::shared_ptr<Renderer> renderer)
	: _renderer(renderer)
{
}

}
