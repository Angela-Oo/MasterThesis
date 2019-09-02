#include "mesh_visualizer.h"

namespace Visualizer {


void MeshVisualizer::visualize(bool visible)
{
	if (visible)
	{
		bool override_mesh = _last_rendered_current_frame != _current_frame;
		//if (_render_mesh == Render::ONLY_DEFORMATION_GRAPH) {
		//	_renderer->removeMesh("mesh");

		//	_renderer->insertMesh("mesh", _mesh_reader->getMesh(_current_frame), 0.001f, false, override_mesh);
		//	_renderer->insertPoints("mesh_p", mesh_reader->getMesh(_current_frame), ml::RGBColor::Green, 0.005f, override_mesh);
		//}
		//else {
		_renderer->insert("mesh", _mesh_reader->getMesh(_current_frame), RenderMode::MESH, ml::RGBColor::White, override_mesh);
		//}
		_last_rendered_current_frame = _current_frame;
	}
	else {
		_renderer->remove("mesh");
	}
}

void MeshVisualizer::setCurrentFrame(unsigned int current_frame)
{
	_current_frame = current_frame;
}

MeshVisualizer::MeshVisualizer(std::shared_ptr<Renderer> renderer,
							   std::shared_ptr<IMeshReader> mesh_reader)
	: _renderer(renderer)
	, _mesh_reader(mesh_reader)
	, _current_frame(0)
	, _last_rendered_current_frame(0)
{}

}
