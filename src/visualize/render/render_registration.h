#pragma once

#include "visualize/visualizer/i_registration_visualizer.h"
#include "visualize/render/renderer.h"
#include <memory>

namespace Visualizer {

class RendererRegistration {
private:
	std::shared_ptr<Renderer> _renderer;
public:
	void renderDeformedSourceMesh(const SurfaceMesh & deformed_points, RegistrationRenderMode mode, bool render_vertex_color);
	void renderTargetMesh(const SurfaceMesh & target, RegistrationRenderMode mode);
	void renderDeformationGraph(const SurfaceMesh & deformation_graph, RegistrationRenderMode mode);
public:
	RendererRegistration(std::shared_ptr<Renderer> renderer);
};


}